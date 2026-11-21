#!/usr/bin/env python3
"""
Control cinemático (espacio articular) del SCARA RRP Robrot, con pinza.

Ley de control, para cada junta i:

    u_i = qdot_ref_i(t) + Kp * (q_ref_i(t) - q_i)
    u_i = sat(u_i, -u_max_i, +u_max_i)

    u_max = 0.20 rad/s  (rotacionales q1, q2)
    u_max = 0.15 m/s    (prismática d3, positivo = baja; 0.085 = home abajo)
    u_max = 1.5 rad/s / 0.05 m/s (servo y dedos de la pinza)

u se envía como comando de VELOCIDAD a /velocity_controller/commands
(Gazebo, o el robot real a través de puente_robot_real.py).

Trayectorias: archivos T*.txt con columnas  t  q1  q2  d3
(se instalan en share/Robrot_description/trayectorias).

Pinza: al terminar las trayectorias indicadas en el parámetro 'pinza_eventos'
(por defecto "T4:2,T7:3") la pinza se ABRE, espera esos segundos y se CIERRA,
y luego sigue con la siguiente trayectoria.

Uso (con Gazebo o con el robot real corriendo):
    ros2 run Robrot_description control_cinematico.py --ros-args -p use_sim_time:=true -p trayectoria:=T4
    ros2 run Robrot_description control_cinematico.py --ros-args -p use_sim_time:=true -p trayectoria:=todas
    (con el robot real: use_sim_time:=false)
"""
import csv
import glob
import math
import os
import re

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from ament_index_python.packages import get_package_share_directory

# Mismo orden que config/gz_controllers.yaml
JOINTS = ['Revoluci_n_6', 'Revoluci_n_7', 'Corredera_9',
          'Revoluci_n_12', 'Corredera_13', 'Corredera_14']
PRISMATIC = {'Corredera_9', 'Corredera_13', 'Corredera_14'}

# Límites físicos de Corredera_9 en el URDF: 0 .. 0.085 m (0 = arriba, 0.085 = abajo/home; positivo = baja).
# Se deja 1 mm de margen: si la referencia queda justo en el tope, Gazebo marca
# la junta "fuera de límites" y le anula el comando de velocidad.
D3_JOINT_MIN, D3_JOINT_MAX = 0.001, 0.084
# Rango de d3 en los archivos de trayectoria
D3_TRAJ_MIN, D3_TRAJ_MAX = 0.01, 0.10

# Pinza: servo 0 = cerrada, pi/2 = abierta. Cada dedo se abre APERTURA*sin(servo).
SERVO_CERRADA, SERVO_ABIERTA = 0.0, math.pi / 2
APERTURA = 0.018


def ref_pinza(servo):
    """Referencia de las 3 juntas de la pinza [servo, dedo1, dedo2] para un ángulo de servo."""
    a = APERTURA * math.sin(min(max(servo, 0.0), math.pi / 2))
    return np.array([servo, a, a])


def leer_eventos(texto):
    """'T4:2,T7:3' -> {'T4': 2.0, 'T7': 3.0}"""
    ev = {}
    for parte in texto.replace(' ', '').split(','):
        if ':' in parte:
            n, s = parte.split(':')
            ev[n] = float(s)
    return ev


class ControlCinematico(Node):
    def __init__(self):
        super().__init__('control_cinematico')
        share = get_package_share_directory('Robrot_description')
        self.declare_parameter('trayectoria', 'T1')            # 'T5' o 'todas'
        self.declare_parameter('carpeta', os.path.join(share, 'trayectorias'))
        self.declare_parameter('kp', 2.0)                        # ganancia [1/s]
        self.declare_parameter('w_max', 0.20)                    # rad/s
        self.declare_parameter('v_max', 0.15)                    # m/s
        self.declare_parameter('w_servo', 1.5)                   # rad/s (servo de la pinza)
        self.declare_parameter('v_dedos', 0.05)                  # m/s (dedos de la pinza)
        self.declare_parameter('escalar_tiempo', True)           # estira t para que qdot_ref <= límite
        self.declare_parameter('modo_d3', 'escalado')            # 'escalado' o 'directo'
        self.declare_parameter('tolerancia', 0.005)
        self.declare_parameter('tolerancia_pinza', 0.05)         # rad del servo
        self.declare_parameter('timeout_pinza', 5.0)             # s para abrir o cerrar
        self.declare_parameter('pinza_eventos', 'T4:2,T7:3')     # trayectoria:segundos abierta
        self.declare_parameter('frecuencia', 50.0)
        self.declare_parameter('carpeta_log', os.path.expanduser('~/scara_logs'))
        self.declare_parameter('timeout_inicio', 15.0)           # s para llegar al punto inicial

        p = lambda n: self.get_parameter(n).value
        self.kp = float(p('kp'))
        self.umax = np.array([p('w_max'), p('w_max'), p('v_max'),
                              p('w_servo'), p('v_dedos'), p('v_dedos')], dtype=float)
        self.tol = float(p('tolerancia'))
        self.tol_pinza = float(p('tolerancia_pinza'))
        self.timeout_pinza = float(p('timeout_pinza'))
        self.timeout_inicio = float(p('timeout_inicio'))
        self.eventos = leer_eventos(p('pinza_eventos'))
        self.servo_ref = None          # ángulo del servo pedido (se toma el actual al arrancar)
        self.t_estado = None
        self.t_log_err = 0.0
        self.escalar = bool(p('escalar_tiempo'))
        self.modo_d3 = p('modo_d3')
        self.log_dir = p('carpeta_log')
        os.makedirs(self.log_dir, exist_ok=True)

        self.cola = self._cargar(p('carpeta'), p('trayectoria'))
        if not self.cola:
            raise RuntimeError('No se encontraron trayectorias')
        if self.eventos:
            self.get_logger().info('Pinza: ' + ', '.join(
                f'al terminar {n} abre y cierra después de {s:g} s' for n, s in self.eventos.items()))

        self.q = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.pub_ref = self.create_publisher(JointState, '/referencia', 10)
        self.pub_pinza = self.create_publisher(Float64, '/pinza/referencia', 10)   # para el robot real
        self.create_timer(1.0 / float(p('frecuencia')), self._loop)

        # esperando -> al_inicio -> siguiendo -> al_final -> [pinza_abre -> pinza_espera -> pinza_cierra] -> ...
        self.estado = 'esperando'
        self.actual = None
        self.t0 = None
        self.log = None
        self.log_f = None

    # ---------- trayectorias ----------
    def _cargar(self, carpeta, cual):
        files = sorted(glob.glob(os.path.join(carpeta, 'T*.txt')),
                       key=lambda s: int(re.findall(r'\d+', os.path.basename(s))[0]))
        if cual != 'todas':
            files = [f for f in files if os.path.basename(f) == f'{cual}.txt']
        cola = []
        for f in files:
            a = np.loadtxt(f)
            t, q1, q2, d3 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
            d3j = self._map_d3(d3)
            Q = np.column_stack([q1, q2, d3j])
            Qd = np.gradient(Q, t, axis=0)
            # factor de tiempo para que la referencia respete los límites de velocidad
            k = 1.0
            if self.escalar:
                k = max(1.0, float(np.max(np.abs(Qd).max(0) / self.umax[:3])))
            cola.append(dict(nombre=os.path.basename(f)[:-4], t=t * k, Q=Q, Qd=Qd / k, k=k))
            self.get_logger().info(
                f'{os.path.basename(f)}: duración {t[-1]:.2f}s -> {t[-1]*k:.2f}s (factor {k:.2f})')
        return cola

    def _map_d3(self, d3):
        if self.modo_d3 == 'escalado':
            s = (d3 - D3_TRAJ_MIN) / (D3_TRAJ_MAX - D3_TRAJ_MIN)
            return D3_JOINT_MIN + s * (D3_JOINT_MAX - D3_JOINT_MIN)
        return np.clip(d3, D3_JOINT_MIN, D3_JOINT_MAX)

    # ---------- ROS ----------
    def _js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.q = np.array([d[j] for j in JOINTS])

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _ref(self, tr, tau):
        """Referencia (q, qdot) de las 6 juntas; la pinza sigue a self.servo_ref."""
        q = np.zeros(6); qd = np.zeros(6)
        q[3:] = ref_pinza(self.servo_ref)
        t = tr['t']
        tau = min(max(tau, 0.0), t[-1])
        for i in range(3):
            q[i] = np.interp(tau, t, tr['Q'][:, i])
            qd[i] = np.interp(tau, t, tr['Qd'][:, i]) if tau < t[-1] else 0.0
        return q, qd

    def _enviar(self, u):
        self.pub.publish(Float64MultiArray(data=[float(x) for x in u]))

    def _cambiar(self, estado, now):
        self.estado = estado
        self.t_estado = now

    def _terminar_trayectoria(self, now):
        """Después de una trayectoria: evento de pinza (si hay) o pasar a la siguiente."""
        nombre = self.actual['nombre']
        if nombre in self.eventos and self.estado == 'al_final':
            self.servo_ref = SERVO_ABIERTA
            self._cambiar('pinza_abre', now)
            self.get_logger().info(f'[{nombre}] Pinza: ABRIENDO')
            return
        if self.cola:
            self._cambiar('esperando', now)
        else:
            self._cambiar('fin', now)
            self._enviar(np.zeros(6))
            self.get_logger().info('Todas las trayectorias completadas.')
            raise SystemExit

    def _loop(self):
        if self.q is None:
            return
        now = self._now()

        if self.servo_ref is None:
            self.servo_ref = float(self.q[3])
            self.get_logger().info('Pose inicial medida: ' + ', '.join(
                f'{j}={v:.4f}' for j, v in zip(JOINTS, self.q)))

        if self.estado == 'esperando':
            self.actual = self.cola.pop(0)
            self._cambiar('al_inicio', now)
            self.get_logger().info(f'[{self.actual["nombre"]}] Yendo al punto inicial...')

        tr = self.actual
        if self.estado == 'al_inicio':
            # Ir al punto inicial: SOLO término proporcional (sin feedforward).
            qr, _ = self._ref(tr, 0.0)
            qdr = np.zeros(6)
        elif self.estado == 'siguiendo':
            qr, qdr = self._ref(tr, now - self.t0)
        else:  # al_final y estados de la pinza: el brazo se queda en el punto final
            qr, qdr = self._ref(tr, tr['t'][-1])

        e = qr - self.q
        u = np.clip(qdr + self.kp * e, -self.umax, self.umax)
        self._enviar(u)
        self.pub_pinza.publish(Float64(data=float(self.servo_ref)))

        ref = JointState(); ref.header.stamp = self.get_clock().now().to_msg()
        ref.name = JOINTS; ref.position = qr.tolist(); ref.velocity = u.tolist()
        self.pub_ref.publish(ref)

        err = np.max(np.abs(e[:3]))
        err_pinza = abs(e[3])
        if self.estado in ('al_inicio', 'al_final') and now - self.t_log_err > 2.0:
            self.t_log_err = now
            self.get_logger().info(
                f'  error q1={e[0]:+.4f} rad  q2={e[1]:+.4f} rad  d3={e[2]:+.4f} m | '
                f'medido d3={self.q[2]:.4f} m  comando u=[{u[0]:+.3f}, {u[1]:+.3f}, {u[2]:+.3f}]')
        if self.estado == 'al_inicio' and err >= self.tol and now - self.t_estado > self.timeout_inicio:
            self.get_logger().warn('No llegó al punto inicial a tiempo; arranco la trayectoria de todas formas.')
            err = 0.0
        if self.estado == 'al_inicio' and err < self.tol:
            self._cambiar('siguiendo', now); self.t0 = now
            path = os.path.join(self.log_dir, f'{tr["nombre"]}.csv')
            self.log_f = open(path, 'w', newline='')
            self.log = csv.writer(self.log_f)
            self.log.writerow(['t', 'q1_ref', 'q2_ref', 'd3_ref', 'q1', 'q2', 'd3',
                               'u1_rad_s', 'u2_rad_s', 'u3_m_s', 'servo_ref', 'servo', 'dedo1', 'dedo2'])
            self.get_logger().info(f'[{tr["nombre"]}] Siguiendo trayectoria ({tr["t"][-1]:.1f}s). Log: {path}')
        if self.log:
            self.log.writerow([f'{now - self.t0:.3f}', *np.round(qr[:3], 5),
                               *np.round(self.q[:3], 5), *np.round(u[:3], 5),
                               round(self.servo_ref, 4), *np.round(self.q[3:], 5)])
        if self.estado == 'siguiendo' and now - self.t0 >= tr['t'][-1]:
            self._cambiar('al_final', now)
            self.get_logger().info(f'[{tr["nombre"]}] Referencia terminada, convergiendo al punto final...')
        if self.estado == 'al_final' and err >= self.tol and now - self.t_estado > self.timeout_inicio:
            self.get_logger().warn('No convergió al punto final a tiempo; sigo con la siguiente.')
            err = 0.0
        if self.estado == 'al_final' and err < self.tol:
            self.get_logger().info(f'[{tr["nombre"]}] Llegó al punto final. Tiempo {now - self.t0:.2f}s')
            self._terminar_trayectoria(now)
            if self.estado != 'al_final' and self.estado.startswith('pinza'):
                return

        # ---------- pinza: abrir, esperar, cerrar ----------
        if self.estado == 'pinza_abre' and (err_pinza < self.tol_pinza or now - self.t_estado > self.timeout_pinza):
            seg = self.eventos[tr['nombre']]
            self._cambiar('pinza_espera', now)
            self.get_logger().info(f'[{tr["nombre"]}] Pinza ABIERTA (servo={self.q[3]:.2f} rad). Cierro en {seg:g} s')
        elif self.estado == 'pinza_espera' and now - self.t_estado >= self.eventos[tr['nombre']]:
            self.servo_ref = SERVO_CERRADA
            self._cambiar('pinza_cierra', now)
            self.get_logger().info(f'[{tr["nombre"]}] Pinza: CERRANDO')
        elif self.estado == 'pinza_cierra' and (err_pinza < self.tol_pinza or now - self.t_estado > self.timeout_pinza):
            self.get_logger().info(f'[{tr["nombre"]}] Pinza CERRADA. Listo. Tiempo total {now - self.t0:.2f}s')
            if self.log_f:
                self.log_f.close(); self.log = None; self.log_f = None
            self.estado = 'pinza_hecha'
            self._terminar_trayectoria(now)

        # cerrar el log cuando la trayectoria termina sin evento de pinza
        if self.estado == 'esperando' and self.log_f:
            self.log_f.close(); self.log = None; self.log_f = None


def main():
    rclpy.init()
    n = ControlCinematico()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        # al salir (Ctrl+C) se manda velocidad cero para que el robot se detenga
        try:
            if rclpy.ok():
                n._enviar(np.zeros(6))
            if n.log_f:
                n.log_f.close()
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
