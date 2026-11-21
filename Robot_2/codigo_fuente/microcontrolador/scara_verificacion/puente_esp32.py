#!/usr/bin/env python3
"""Puente ROS 2 <-> robot real (ESP32 con scara_verificacion.ino).

- Escucha /scara_controller/commands (Float64MultiArray [hombro_rad, codo_rad, eje_z_m]),
  el MISMO topic que usa Gazebo, y se lo manda al ESP32 como "gz H C D".
  Asi terminal_ik_interface.py / scara_ik_node.py mueven el robot real igual que la simulacion.
- Publica la posicion real (encoders) en /scara_real/joint_states (sensor_msgs/JointState),
  con los nombres de joints del URDF: hombro, codo, eje_z.

Uso (Linux):   ros2 run Scara_Final_description puente_esp32.py --ros-args -p puerto:=/dev/ttyUSB0
     (Windows): ... -p puerto:=COM6
Parametros:  puerto (str), baudios (int, 115200), home_al_iniciar (bool, false),
             topic_estado (str, /scara_real/joint_states)
Necesita pyserial:  pip install pyserial
"""
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import serial
except ImportError:  # pragma: no cover
    raise SystemExit('Falta pyserial: pip install pyserial')

Z0_GZ = 0.0703          # altura de la punta (m) con eje_z = 0 (igual que en el firmware y el URDF)
JOINTS = ['hombro', 'codo', 'eje_z']


class PuenteEsp32(Node):
    def __init__(self):
        super().__init__('puente_esp32')
        self.declare_parameter('puerto', '/dev/ttyUSB0')
        self.declare_parameter('baudios', 115200)
        self.declare_parameter('home_al_iniciar', False)
        self.declare_parameter('topic_estado', '/scara_real/joint_states')

        puerto = self.get_parameter('puerto').value
        baudios = self.get_parameter('baudios').value
        # sin DTR/RTS para no reiniciar el ESP32 al abrir el puerto
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = puerto, baudios, 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.lock = threading.Lock()
        self.get_logger().info(f'Conectado a {puerto}')

        self.pub = self.create_publisher(JointState, self.get_parameter('topic_estado').value, 10)
        self.create_subscription(Float64MultiArray, '/scara_controller/commands', self.comando, 10)

        self.enviar('rep 1')
        if self.get_parameter('home_al_iniciar').value:
            self.enviar('home')
        self.hilo = threading.Thread(target=self.leer, daemon=True)
        self.hilo.start()

    def enviar(self, texto):
        with self.lock:
            self.ser.write((texto + '\n').encode())
        self.get_logger().info(f'-> ESP32: {texto}')

    def comando(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warn('Se esperan 3 valores: [hombro_rad, codo_rad, eje_z_m]')
            return
        h, c, d = msg.data[:3]
        self.enviar(f'gz {h:.5f} {c:.5f} {d:.5f}')

    def leer(self):
        buf = b''
        while rclpy.ok():
            try:
                buf += self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Puerto serie: {e}')
                return
            while b'\n' in buf:
                linea, buf = buf.split(b'\n', 1)
                texto = linea.decode(errors='ignore').strip()
                if texto.startswith('@,'):
                    self.estado(texto)
                elif texto:
                    self.get_logger().info(f'ESP32: {texto}')

    def estado(self, texto):
        # @,estado,q1_deg,q2_deg,z_mm,t1,t2,tz,x_mm,y_mm,homed,fc
        p = texto.split(',')
        try:
            q1, q2, z = float(p[2]), float(p[3]), float(p[4])
        except (IndexError, ValueError):
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [math.radians(q1), math.radians(q2), z / 1000.0 - Z0_GZ]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    nodo = PuenteEsp32()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.enviar('s')
    nodo.ser.close()
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
