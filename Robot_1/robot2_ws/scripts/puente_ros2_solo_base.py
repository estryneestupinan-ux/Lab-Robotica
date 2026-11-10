#!/usr/bin/env python3
"""ROS 2 Lyrical -> ESP32: solo joint_1, sin ordenes de habilitacion.
Requiere: python3-serial, rclpy, control_msgs.
Firmware esperado: main_receptor_solo_base.py.
"""
import argparse
import math
import time
import serial
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

class PuenteBase(Node):
    def __init__(self, puerto, baudios, hz):
        super().__init__('puente_trayectoria_solo_base')
        self.esp = serial.Serial(puerto, baudios, timeout=0, write_timeout=0.2)
        time.sleep(2)
        self.esp.reset_input_buffer()
        self.ref = None
        self.seq = 0
        self.buf = bytearray()
        self.sub = self.create_subscription(
            JointTrajectoryControllerState, '/arm_controller/controller_state',
            self.referencia, 10)
        self.timer = self.create_timer(1.0 / hz, self.enviar)
        self.get_logger().info('Solo joint_1 -> ESP32, sin ARM ni PWM; CTRL+C para STOP.')

    def referencia(self, msg):
        if 'joint_1' not in msg.joint_names:
            return
        i = list(msg.joint_names).index('joint_1')
        if len(msg.reference.positions) <= i:
            return
        val = math.degrees(msg.reference.positions[i])
        if math.isfinite(val):
            self.ref = val

    def enviar(self):
        if self.ref is not None:
            if -160 <= self.ref <= 40:
                self.seq += 1
                try:
                    self.esp.write(('BASE_REF,%d,%.4f\n' %
                                    (self.seq, self.ref)).encode())
                except (serial.SerialException, serial.SerialTimeoutException) as exc:
                    self.get_logger().error('Serial: %s' % exc)
                    self.ref = None
            else:
                self.get_logger().error('Referencia fuera de limite: %.2f' % self.ref)
                self.ref = None
        try:
            data = self.esp.read(512)
            self.buf.extend(data)
            while b'\n' in self.buf:
                raw, _, self.buf = self.buf.partition(b'\n')
                msg = raw.decode(errors='replace').strip()
                if msg.startswith(('ERR,', 'STOP,', 'STATUS,')):
                    self.get_logger().info('ESP32: ' + msg)
                # ACK valido: la referencia se recibio, no implica movimiento.
        except serial.SerialException as exc:
            self.get_logger().error('Lectura serial: %s' % exc)

    def cerrar(self):
        try:
            self.esp.write(b'STOP\n')
            self.esp.flush()
        finally:
            self.esp.close()

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--hz', type=float, default=20.0)
    args = ap.parse_args()
    if args.hz <= 0:
        ap.error('--hz debe ser positivo')
    rclpy.init()
    nodo = None
    try:
        nodo = PuenteBase(args.port, args.baud, args.hz)
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo:
            nodo.cerrar()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
