#!/usr/bin/env python3
"""
ESPEJO Gazebo -> robot real (ESP32 con scara_verificacion.ino).

Gazebo es el "maestro": este nodo lee la pose del robot simulado en /joint_states y se la manda
al ESP32 como posición objetivo ("qq hombro codo z") 20 veces por segundo. El robot real copia
lo que hace Gazebo, sin integrar velocidades (no se desfasa aunque Gazebo vaya más lento que el
tiempo real).

La posición REAL (encoders) se publica en /robot_real/joint_states, para no chocar con Gazebo.

Seguridad:
  - No manda nada hasta que el ESP32 reporte que hizo home (requiere_home:=true).
  - El objetivo avanza como máximo w_max (rad/s) y v_max (m/s): si Gazebo y el robot real están
    lejos al arrancar, el real se acerca despacio en vez de saltar.
  - q1, q2 y d3 se recortan a los límites del robot real.
  - Ctrl+C manda "s" (stop) al ESP32.

No modifica el paquete Robrot_description: usa las mismas conversiones que puente_robot_real.py.
NO corras puente_robot_real.py al mismo tiempo (los dos usarían el puerto serie).

Uso (con Gazebo ya abierto):
    source ~/ros2_ws/install/setup.bash
    python3 ~/scara_tools/espejo_gazebo_real.py --ros-args -p puerto:=/dev/ttyUSB0 -p home_al_iniciar:=true
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:
    raise SystemExit('Falta pyserial:  sudo apt install python3-serial')

NOMBRES = ['Revoluci_n_6', 'Revoluci_n_7', 'Corredera_9',
           'Revoluci_n_12', 'Corredera_13', 'Corredera_14']
APERTURA = 0.018   # m que abre cada dedo con el servo a pi/2


class Espejo(Node):
    def __init__(self):
        super().__init__('espejo_gazebo_real')
        d = self.declare_parameter
        d('puerto', '/dev/ttyUSB0')
        d('baudios', 115200)
        d('home_al_iniciar', True)     # al abrir el USB el ESP32 se reinicia y pierde el home
        d('requiere_home', True)
        # Conversión modelo <-> firmware (mismos valores que puente_robot_real.py)
        d('offset_hombro', -138.11)     # grados del firmware con Revoluci_n_6 = 0
        d('offset_codo', -46.47)        # grados del firmware con Revoluci_n_7 = 0
        d('z_punta_d0', 247.0)          # mm de la punta con Corredera_9 = 0
        d('servo_cerrada', 25.0)
        d('servo_abierta', 145.0)
        # Límites del robot real (en el modelo)
        # (del firmware: hombro [-212, -92] y codo [-92, 88] grados -> en el modelo, con los offsets)
        d('q1_min_deg', -73.9)
        d('q1_max_deg', 46.1)
        d('q2_min_deg', -45.5)
        d('q2_max_deg', 90.0)
        d('d3_min', 0.0)
        d('d3_max', 0.085)
        # Velocidad máxima con la que se mueve el objetivo del robot real
        d('w_max', 0.20)                # rad/s
        d('v_max', 0.15)                # m/s
        d('frecuencia_envio', 20.0)     # Hz
        d('topico_sim', '/joint_states')
        d('topico_real', '/robot_real/joint_states')
        g = lambda n: self.get_parameter(n).value

        self.off_h, self.off_c, self.z0 = float(g('offset_hombro')), float(g('offset_codo')), float(g('z_punta_d0'))
        self.s_cerr, self.s_abie = float(g('servo_cerrada')), float(g('servo_abierta'))
        self.lo = [math.radians(g('q1_min_deg')), math.radians(g('q2_min_deg')), float(g('d3_min'))]
        self.hi = [math.radians(g('q1_max_deg')), math.radians(g('q2_max_deg')), float(g('d3_max'))]
        self.vmax = [float(g('w_max')), float(g('w_max')), float(g('v_max'))]
        self.dt = 1.0 / float(g('frecuencia_envio'))
        self.requiere_home = bool(g('requiere_home'))

        url = g('puerto')
        self.ser = serial.serial_for_url(url, baudrate=int(g('baudios')), timeout=0.05, do_not_open=True)
        try:   # sin DTR/RTS para no reiniciar el ESP32 al abrir el USB
            self.ser.dtr = False
            self.ser.rts = False
        except (AttributeError, ValueError, serial.SerialException):
            pass
        self.ser.open()
        self.lock = threading.Lock()
        self.get_logger().info(f'Conectado al ESP32 en {url}')
        # En Linux, abrir el USB reinicia el ESP32 (se ve "rst:0x1 POWERON_RESET").
        # Hay que esperar a que arranque; si no, los primeros comandos se pierden.
        self.get_logger().info('Esperando a que el ESP32 arranque (3 s)...')
        time.sleep(3.0)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        self.q_sim = None        # [q1, q2, d3] de Gazebo
        self.servo_sim = 0.0     # rad (Revoluci_n_12 de Gazebo)
        self.q_med = None        # [q1, q2, d3] medidos en el robot real
        self.homed = False
        self.estado_fw = ''
        self.q_obj = None        # objetivo que se le manda al real
        self.ultimo = ''
        self.servo_enviado = None
        self.t_aviso = 0.0
        self.t_ultimo_estado = time.monotonic()
        self.t_rep = 0.0

        self.pub_real = self.create_publisher(JointState, g('topico_real'), 10)
        self.create_subscription(JointState, g('topico_sim'), self._sim, 10)
        self.create_timer(self.dt, self._enviar)

        self.tx('rep 1')
        if g('home_al_iniciar'):
            self.tx('reset')      # por si quedó en FALLA
            self.tx('home')
        threading.Thread(target=self._leer, daemon=True).start()

    # ---------- conversión ----------
    def a_fw(self, q1, q2, d3):
        return math.degrees(q1) + self.off_h, math.degrees(q2) + self.off_c, self.z0 - 1000.0 * d3

    def a_modelo(self, h, c, z):
        return [math.radians(h - self.off_h), math.radians(c - self.off_c), (self.z0 - z) / 1000.0]

    # ---------- serie ----------
    def tx(self, texto, log=True):
        with self.lock:
            self.ser.write((texto + '\n').encode())
        if log:
            self.get_logger().info(f'-> ESP32: {texto}')

    # ---------- Gazebo ----------
    def _sim(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in NOMBRES[:3]):
            self.q_sim = [d[n] for n in NOMBRES[:3]]
        if 'Revoluci_n_12' in d:
            self.servo_sim = d['Revoluci_n_12']

    def _enviar(self):
        ahora = time.monotonic()
        if ahora - self.t_ultimo_estado > 2.0 and ahora - self.t_rep > 2.0:
            self.t_rep = ahora
            self.tx('rep 1', log=False)          # por si el ESP32 no recibió el primero
        if self.q_sim is None or self.q_med is None:
            if ahora - self.t_aviso > 3.0:
                self.t_aviso = ahora
                falta = 'Gazebo (/joint_states)' if self.q_sim is None else 'el estado del ESP32 (líneas @)'
                self.get_logger().warning(f'Esperando {falta}...')
            return
        if self.estado_fw == 'FALLA':
            if ahora - self.t_aviso > 3.0:
                self.t_aviso = ahora
                self.get_logger().error('El ESP32 está en FALLA (mira el mensaje "!!! FALLA" arriba). '
                                        'Corrige la causa y reinicia este nodo con -p home_al_iniciar:=true.')
            self.q_obj = None
            return
        if self.requiere_home and not self.homed:
            if ahora - self.t_aviso > 3.0:
                self.t_aviso = ahora
                self.get_logger().warning('El ESP32 no ha hecho home: no muevo nada (usa -p home_al_iniciar:=true).')
            self.q_obj = None
            return
        if self.q_obj is None:
            self.q_obj = list(self.q_med)
            self.get_logger().info('Siguiendo a Gazebo.')

        for i in range(3):
            meta = min(max(self.q_sim[i], self.lo[i]), self.hi[i])
            paso = self.vmax[i] * self.dt
            self.q_obj[i] += min(max(meta - self.q_obj[i], -paso), paso)

        h, c, z = self.a_fw(*self.q_obj)
        texto = f'qq {h:.2f} {c:.2f} {z:.2f}'
        if texto != self.ultimo:
            self.tx(texto, log=False)
            self.ultimo = texto

        s = min(max(self.servo_sim, 0.0), math.pi / 2)
        grados = self.s_cerr + (self.s_abie - self.s_cerr) * s / (math.pi / 2)
        if self.servo_enviado is None or abs(grados - self.servo_enviado) >= 1.0:
            self.tx(f'pinza {grados:.0f}')
            self.servo_enviado = grados

    # ---------- ESP32 ----------
    def _leer(self):
        buf = b''
        while rclpy.ok():
            try:
                buf += self.ser.read(256)
            except Exception as e:          # puerto cerrado al salir, cable desconectado, etc.
                if self.ser.is_open:
                    self.get_logger().error(f'Conexión con el ESP32: {e}')
                return
            while b'\n' in buf:
                linea, buf = buf.split(b'\n', 1)
                t = linea.decode(errors='ignore').strip()
                if t.startswith('@,'):
                    self._estado(t)
                elif t:
                    self.get_logger().info(f'ESP32: {t}')

    def _estado(self, t):
        # @,estado,hombro_deg,codo_deg,z_mm,t1,t2,tz,x_mm,y_mm,homed(3 dígitos),fc(3 dígitos)
        p = t.split(',')
        try:
            self.q_med = self.a_modelo(float(p[2]), float(p[3]), float(p[4]))
            self.t_ultimo_estado = time.monotonic()
            self.estado_fw = p[1].strip()
            self.homed = p[10].strip() == '111'      # hombro, codo y z con home
        except (IndexError, ValueError):
            return
        s = min(max(self.servo_sim, 0.0), math.pi / 2)
        a = APERTURA * math.sin(s)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = NOMBRES
        js.position = [*self.q_med, s, a, a]
        self.pub_real.publish(js)


def main():
    rclpy.init()
    n = Espejo()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.tx('s')
            time.sleep(0.1)
            n.ser.close()
        except Exception:
            pass
        try:
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
