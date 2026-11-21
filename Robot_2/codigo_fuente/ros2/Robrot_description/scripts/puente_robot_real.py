#!/usr/bin/env python3
"""
Puente ROS 2 <-> robot real (ESP32 con scara_verificacion.ino).

Hace que el robot real se comporte como el de Gazebo para control_cinematico.py:
  - Escucha /velocity_controller/commands (las 6 velocidades, mismo orden que gz_controllers.yaml),
    las integra y le manda al ESP32 la posición objetivo ("qq hombro codo z", 20 veces por segundo).
  - Escucha /pinza/referencia (rad del servo: 0 cerrada, pi/2 abierta) y mueve el servo real ("pinza G").
  - Lee la posición real de los encoders del ESP32 y la publica en /joint_states con los
    nombres del URDF (Revoluci_n_6, Revoluci_n_7, Corredera_9, Revoluci_n_12, Corredera_13, Corredera_14).

NO lances Gazebo al mismo tiempo (los dos publicarían /joint_states).

Uso:
    ros2 run Robrot_description puente_robot_real.py --ros-args -p puerto:=/dev/ttyUSB0 -p home_al_iniciar:=true
    ros2 run Robrot_description control_cinematico.py --ros-args -p trayectoria:=T4
El puerto también puede ser una dirección de red, p.ej. -p puerto:=socket://192.168.1.50:3333
Necesita pyserial:  pip install pyserial   (o: sudo apt install python3-serial)

Conversión modelo <-> firmware (ángulos del firmware: hombro medido desde el eje x del mundo,
codo medido desde el brazo estirado; z = altura de la punta de la cremallera en mm):
    hombro_fw[grados] = grad(Revoluci_n_6) + offset_hombro          (offset_hombro = -138.11 en el CAD)
    codo_fw[grados]   = grad(Revoluci_n_7) + offset_codo            (offset_codo   =  -46.47 en el CAD)
    z_fw[mm]          = z_punta_d0 - 1000 * Corredera_9             (z_punta_d0 = 247: robot real, 162 mm en el home con d3 = 0.085)
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

try:
    import serial
except ImportError:  # pragma: no cover
    raise SystemExit('Falta pyserial:  pip install pyserial   (o: sudo apt install python3-serial)')

JOINTS = ['Revoluci_n_6', 'Revoluci_n_7', 'Corredera_9',
          'Revoluci_n_12', 'Corredera_13', 'Corredera_14']
APERTURA = 0.018                 # m que abre cada dedo con el servo a 90 grados
D3_MIN, D3_MAX = 0.0, 0.085      # límites de Corredera_9 en el URDF (0.085 = abajo, home de Z)


class Conversion:
    """Pasa de las juntas del modelo (rad, m) a las del firmware (grados, mm) y al revés."""

    def __init__(self, off_h, off_c, z0):
        self.off_h, self.off_c, self.z0 = off_h, off_c, z0

    def a_firmware(self, q1, q2, d3):
        return math.degrees(q1) + self.off_h, math.degrees(q2) + self.off_c, self.z0 - 1000.0 * d3

    def a_modelo(self, h, c, z):
        return math.radians(h - self.off_h), math.radians(c - self.off_c), (self.z0 - z) / 1000.0


class PuenteRobotReal(Node):
    def __init__(self):
        super().__init__('puente_robot_real')
        d = self.declare_parameter
        d('puerto', '/dev/ttyUSB0')
        d('baudios', 115200)
        d('home_al_iniciar', False)
        d('offset_hombro', -138.11)       # grados (ver docstring)
        d('offset_codo', -46.47)          # grados
        d('z_punta_d0', 247.0)           # mm: robot real, punta a 162 mm con d3 = 0.085 (home de Z, abajo)
        d('servo_cerrada', 25.0)         # grados del servo real con la pinza cerrada (medido)
        d('servo_abierta', 145.0)        # grados del servo real con la pinza abierta (medido)
        d('frecuencia_envio', 20.0)      # Hz
        p = lambda n: self.get_parameter(n).value

        self.conv = Conversion(float(p('offset_hombro')), float(p('offset_codo')), float(p('z_punta_d0')))
        self.servo_cerr, self.servo_abie = float(p('servo_cerrada')), float(p('servo_abierta'))

        url = p('puerto')
        # sin DTR/RTS para no reiniciar el ESP32 al abrir el puerto USB
        self.ser = serial.serial_for_url(url, baudrate=int(p('baudios')), timeout=0.05, do_not_open=True)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except (AttributeError, ValueError, serial.SerialException):
            pass
        self.ser.open()
        self.lock = threading.Lock()
        self.get_logger().info(f'Conectado al ESP32 en {url}')

        self.q_med = None                # [q1, q2, d3] medidos (modelo)
        self.q_obj = None                # objetivo integrado
        self.servo = 0.0                 # rad pedido a la pinza
        self.servo_enviado = None
        self.t_cmd = None
        self.ultimo_envio = ''

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Float64MultiArray, '/velocity_controller/commands', self._velocidades, 10)
        self.create_subscription(Float64, '/pinza/referencia', self._pinza, 10)
        self.create_timer(1.0 / float(p('frecuencia_envio')), self._enviar_objetivo)

        self.enviar('rep 1')
        if p('home_al_iniciar'):
            self.enviar('home')
        self._pinza(Float64(data=0.0))   # empieza cerrada
        threading.Thread(target=self._leer, daemon=True).start()

    # ---------- envio al ESP32 ----------
    def enviar(self, texto, log=True):
        with self.lock:
            self.ser.write((texto + '\n').encode())
        if log:
            self.get_logger().info(f'-> ESP32: {texto}')

    def _velocidades(self, msg):
        if self.q_med is None or len(msg.data) < 3:
            return
        ahora = time.monotonic()
        if self.q_obj is None or self.t_cmd is None:
            self.q_obj = list(self.q_med)
            self.t_cmd = ahora
            return
        dt = min(ahora - self.t_cmd, 0.1)
        self.t_cmd = ahora
        for i in range(3):
            self.q_obj[i] += float(msg.data[i]) * dt
        self.q_obj[2] = min(max(self.q_obj[2], D3_MIN), D3_MAX)

    def _enviar_objetivo(self):
        if self.q_obj is None:
            return
        h, c, z = self.conv.a_firmware(*self.q_obj)
        texto = f'qq {h:.2f} {c:.2f} {z:.2f}'
        if texto != self.ultimo_envio:
            self.enviar(texto, log=False)
            self.ultimo_envio = texto

    def _pinza(self, msg):
        self.servo = min(max(float(msg.data), 0.0), math.pi / 2)
        g = self.servo_cerr + (self.servo_abie - self.servo_cerr) * self.servo / (math.pi / 2)
        if self.servo_enviado is None or abs(g - self.servo_enviado) >= 1.0:
            self.enviar(f'pinza {g:.0f}')
            self.servo_enviado = g

    # ---------- lectura del ESP32 ----------
    def _leer(self):
        buf = b''
        while rclpy.ok():
            try:
                buf += self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Conexión con el ESP32: {e}')
                return
            while b'\n' in buf:
                linea, buf = buf.split(b'\n', 1)
                texto = linea.decode(errors='ignore').strip()
                if texto.startswith('@,'):
                    self._estado(texto)
                elif texto:
                    self.get_logger().info(f'ESP32: {texto}')

    def _estado(self, texto):
        # @,estado,hombro_deg,codo_deg,z_mm,t1,t2,tz,x_mm,y_mm,homed,fc
        p = texto.split(',')
        try:
            q1, q2, d3 = self.conv.a_modelo(float(p[2]), float(p[3]), float(p[4]))
        except (IndexError, ValueError):
            return
        self.q_med = [q1, q2, d3]
        a = APERTURA * math.sin(self.servo)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [q1, q2, d3, self.servo, a, a]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    nodo = PuenteRobotReal()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            nodo.enviar('s')
            nodo.ser.close()
        except Exception:
            pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
