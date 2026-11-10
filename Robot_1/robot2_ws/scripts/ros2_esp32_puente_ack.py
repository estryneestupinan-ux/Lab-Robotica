#!/usr/bin/env python3
"""Puente ROS2 -> ESP32, solo prueba sin motores; lee ACK no bloqueante.
Usar con esp32_integracion_modo_seguro.py, NO ejecuta el robot fisico.
"""
import argparse
import math
import time
import serial
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

NOMBRES = ('joint_1', 'joint_2', 'joint_3', 'joint_4')

class Nodo(Node):
    def __init__(self, port, baud, hz, confirmar_home=False):
        super().__init__('puente_esp32_ack')
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.2)
        self.seq = 0
        self.last = 0.0
        self.periodo = 1.0 / hz
        self.last_ack = -1
        self.buffer = b''
        self.create_subscription(JointTrajectoryControllerState,
                                 '/arm_controller/controller_state', self.rx_ros, 10)
        self.create_timer(0.03, self.rx_serial)
        self.get_logger().info('Solo prueba: SIN movimiento fisico. Puerto %s' % port)
        if confirmar_home:
            time.sleep(2)  # puerto USB puede reiniciar MicroPython
            self.ser.write(b'HOME_CONFIRM\n')
            self.get_logger().info('Enviado HOME_CONFIRM (referencia logica, NO sensor)')
        self.get_logger().info('Confirmar HOME solo si la postura fisica coincide con Gazebo.')

    def rx_ros(self, msg):
        t = time.monotonic()
        if t - self.last < self.periodo:
            return
        try:
            idx = [msg.joint_names.index(n) for n in NOMBRES]
            q = [math.degrees(msg.reference.positions[i]) for i in idx]
            if not all(math.isfinite(x) for x in q):
                return
            trama = 'REF,%d,%s\n' % (self.seq, ','.join('%.4f' % x for x in q))
            self.ser.write(trama.encode('ascii'))
            self.last = t
            if self.seq % 20 == 0:
                self.get_logger().info('TX '+trama.strip())
            self.seq += 1
        except (ValueError, IndexError):
            self.get_logger().warning('Faltan nombres o referencias en controller_state')
        except serial.SerialException as err:
            self.get_logger().error('Error serial: %s' % err)
            raise

    def rx_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.buffer += self.ser.read(min(n, 2048))
                while b'\n' in self.buffer:
                    linea, self.buffer = self.buffer.split(b'\n', 1)
                    texto = linea.decode('utf-8', errors='replace').strip()
                    if texto.startswith('ACK,'):
                        try:
                            self.last_ack = int(texto.split(',')[1])
                        except ValueError:
                            pass
                        if self.last_ack % 20 == 0:
                            self.get_logger().info('ESP32 '+texto)
                    elif texto:
                        self.get_logger().info('ESP32 '+texto)
        except serial.SerialException as err:
            self.get_logger().error('Error al leer ESP32: %s' % err)
            raise

    def cerrar(self):
        self.ser.close()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--hz', type=float, default=20)
    ap.add_argument('--confirmar-home', action='store_true', help='Solo si el robot esta fisicamente en HOME')
    args=ap.parse_args()
    if not 0 < args.hz <= 20:
        ap.error('hz debe ser >0 y <=20')
    rclpy.init()
    nodo=None
    try:
        nodo=Nodo(args.port,args.baud,args.hz,args.confirmar_home)
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo: nodo.cerrar(); nodo.destroy_node()
        rclpy.shutdown()
if __name__=='__main__': main()
