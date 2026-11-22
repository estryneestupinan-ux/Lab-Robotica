#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo ROS 2 (Jazzy) que ejecuta en Gazebo la tarea HOME -> PLL -> PS -> HOME
segun la planeacion de trayectorias de tareas_tabla.py.

 - Envia cada tramo como goal FollowJointTrajectory a arm_controller / gripper_controller.
 - Sujeta y suelta el cubo con el sistema DetachableJoint de Gazebo (/cubo/attach, /cubo/detach).
 - Guarda la trayectoria planeada y la real (/joint_states) en CSV y genera las graficas
   en ~/scara_resultados.
"""
import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import planificador_trayectorias as pt  # noqa: E402
import tareas_tabla as tt  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402
try:
    from ros_gz_interfaces.msg import Contacts  # noqa: E402
except ImportError:  # sin ros_gz_interfaces no hay monitor de choques
    Contacts = None

ENLACES_SENSOR = ['eslabon1', 'eslabon2', 'prismatico']


class EjecutorTareas(Node):
    def __init__(self):
        super().__init__('ejecutar_tareas')
        self.declare_parameter('perfil', 'quintico')
        self.declare_parameter('espera', 3.0)
        self.declare_parameter('carpeta_resultados', os.path.expanduser('~/scara_resultados'))
        self.perfil = self.get_parameter('perfil').value
        self.espera = float(self.get_parameter('espera').value)
        self.carpeta = os.path.expanduser(self.get_parameter('carpeta_resultados').value)
        if self.perfil not in pt.PERFILES:
            self.get_logger().warn('Perfil "%s" desconocido, se usa quintico' % self.perfil)
            self.perfil = 'quintico'

        self.cli = {
            'brazo': ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'),
            'garra': ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'),
        }
        self.pub_attach = self.create_publisher(Empty, '/cubo/attach', 10)
        self.pub_detach = self.create_publisher(Empty, '/cubo/detach', 10)
        self.real, self.deseado, self.idx = [], [], None
        self.create_subscription(JointState, '/joint_states', self._cb_js, 50)

        # ---- monitor de choques (sensores de contacto de Gazebo) ----
        self.choques, self.ultimo_choque, self.tramo_actual = [], {}, 'inicio'
        if Contacts is not None:
            for l in ENLACES_SENSOR:
                self.create_subscription(Contacts, '/choques/' + l, self._cb_choque, 10)
        else:
            self.get_logger().warning('ros_gz_interfaces no disponible: monitor de choques desactivado')

    # ------------------------------------------------------------------
    def _cb_js(self, msg):
        if self.idx is None:
            nombres = pt.ARM_JOINTS + pt.GRIPPER_JOINTS
            if not all(n in msg.name for n in nombres):
                return
            self.idx = [msg.name.index(n) for n in nombres]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.real.append([t] + [msg.position[i] for i in self.idx])

    @staticmethod
    def _nombre(entidad):
        n = entidad.name or str(entidad.id)
        partes = [p for p in n.split('::') if p]
        if len(partes) >= 2:
            return partes[-2] if partes[0] == 'urdfscarafinal' else partes[0]
        return n

    def _cb_choque(self, msg):
        ahora = time.monotonic()
        for c in msg.contacts:
            par = tuple(sorted((self._nombre(c.collision1), self._nombre(c.collision2))))
            if ahora - self.ultimo_choque.get(par, -10.0) > 1.0:
                self.get_logger().warning('*** CHOQUE: %s  <->  %s   (tramo: %s)'
                                          % (par[0], par[1], self.tramo_actual))
                self.choques.append([self._ahora(), par[0], par[1], self.tramo_actual])
            self.ultimo_choque[par] = ahora

    def _ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _esperar(self, segundos):
        fin = time.monotonic() + segundos
        while time.monotonic() < fin and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publicar(self, pub, veces=3):
        for _ in range(veces):
            pub.publish(Empty())
            self._esperar(0.1)

    # ------------------------------------------------------------------
    def _enviar(self, seg):
        es_brazo = seg['tipo'] == 'brazo'
        cli = self.cli['brazo' if es_brazo else 'garra']
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = pt.ARM_JOINTS if es_brazo else pt.GRIPPER_JOINTS
        for k in range(1, len(seg['t'])):          # el punto 0 es la posicion actual
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in seg['q'][k]]
            p.velocities = [float(v) for v in seg['qd'][k]]
            p.accelerations = [float(v) for v in seg['qdd'][k]]
            p.time_from_start = Duration(seconds=float(seg['t'][k])).to_msg()
            goal.trajectory.points.append(p)

        t0 = self._ahora()
        n = len(self.deseado)
        for k, tk in enumerate(seg['t']):
            q = seg['q'][k]
            fila = list(q) + [np.nan] if es_brazo else [np.nan] * 3 + [q[0]]
            self.deseado.append([t0 + tk] + fila + [n])

        fut = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('El controlador rechazo el tramo: %s' % seg['desc'])
            return
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=float(seg['t'][-1]) + 15.0)
        if not res.done():
            self.get_logger().warn('Tiempo excedido en: %s' % seg['desc'])
        elif res.result().result.error_code != 0:
            r = res.result().result
            self.get_logger().warn('Error %d (%s) en: %s' % (r.error_code, r.error_string, seg['desc']))

    def ejecutar(self):
        log = self.get_logger()
        for nombre, c in self.cli.items():
            log.info('Esperando el controlador del %s ...' % nombre)
            while not c.wait_for_server(timeout_sec=2.0) and rclpy.ok():
                log.info('  ... aun esperando')

        segs = tt.construir(self.perfil)
        dur = sum(s['t'][-1] for s in segs)
        log.info('=' * 62)
        log.info(' PLANEACION: %d tramos, perfil %s, duracion %.1f s' % (len(segs), self.perfil, dur))
        for alias, xyz, ang, q, e in tt.resumen():
            log.info('  %-11s tabla %s  ->  q = [%.1f deg, %.1f deg, %.1f cm]'
                     % (alias, xyz, np.rad2deg(q[0]), np.rad2deg(q[1]), q[2] * 100))
        log.info('=' * 62)

        self._esperar(self.espera)
        self._publicar(self.pub_detach)          # el cubo arranca libre sobre su mesa

        # si el robot no arranco en HOME, primero se lleva a HOME
        q_home = tt.a_articular('HOME')
        if self.real:
            q_act = np.array(self.real[-1][1:4])
            if np.max(np.abs(q_act - q_home)) > 0.02:
                log.info('El robot no esta en HOME: moviendolo a HOME primero')
                t, Q, QD, QDD = pt.tray_articular(q_act, q_home, self.perfil, vmax=tt.VMAX)
                self._enviar(dict(tipo='brazo', t=t, q=Q, qd=QD, qdd=QDD, desc='Ir a HOME', evento=None))
        self.choques = []                        # solo cuentan los choques durante la tarea

        for i, seg in enumerate(segs, 1):
            if not rclpy.ok():
                return
            log.info('[%2d/%d] %-36s (%.2f s)' % (i, len(segs), seg['desc'], seg['t'][-1]))
            self.tramo_actual = seg['desc']
            ev = seg['evento']
            if ev and ev[0] == 'soltar':
                self._publicar(self.pub_detach)
                log.info('        cubo liberado')
            self._enviar(seg)
            if ev and ev[0] == 'sujetar':
                self._publicar(self.pub_attach)
                log.info('        cubo sujeto')
        log.info('Tarea completada.')
        if self.choques:
            log.warning('Se detectaron %d choques (ver choques_%s.csv)' % (len(self.choques), self.perfil))
        else:
            log.info('No se detectaron choques del brazo con la celda.')
        self._esperar(1.0)
        self.guardar(segs)

    def guardar(self, segs):
        log = self.get_logger()
        os.makedirs(self.carpeta, exist_ok=True)
        f_des = os.path.join(self.carpeta, 'deseado_%s.csv' % self.perfil)
        f_real = os.path.join(self.carpeta, 'real_%s.csv' % self.perfil)
        np.savetxt(f_des, np.array(self.deseado), delimiter=',', header='t,q1,q2,d3,garra,segmento', comments='')
        np.savetxt(f_real, np.array(self.real), delimiter=',', header='t,q1,q2,d3,garra1,garra2', comments='')
        with open(os.path.join(self.carpeta, 'choques_%s.csv' % self.perfil), 'w') as f:
            f.write('t,objeto1,objeto2,tramo\n')
            for c in self.choques:
                f.write('%.3f,%s,%s,%s\n' % tuple(c))
        log.info('Datos guardados en %s' % self.carpeta)
        try:
            import graficar_resultados
            log.info('Grafica: %s' % graficar_resultados.graficar(f_des, f_real, self.carpeta, self.perfil))
            for f in tt.graficar(segs, self.carpeta, self.perfil):
                log.info('Grafica: %s' % f)
        except Exception as e:
            log.warning('No se pudieron generar las graficas: %s' % e)


def main():
    rclpy.init()
    nodo = EjecutorTareas()
    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
