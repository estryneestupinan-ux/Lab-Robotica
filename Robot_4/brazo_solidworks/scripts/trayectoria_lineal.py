#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import csv
import os

# Parámetros geométricos CAD (mm)
L1 = 120.0
L2 = 160.0
H_BASE = 163.0
D3_MAX = 105.0

class TrayectoriaLinealNode(Node):
    def __init__(self):
        super().__init__('trayectoria_lineal')

        # Publicadores
        self.pub_q1 = self.create_publisher(Float64MultiArray, '/joint1_position_controller/commands', 10)
        self.pub_q2 = self.create_publisher(Float64MultiArray, '/joint2_position_controller/commands', 10)
        self.pub_q3 = self.create_publisher(Float64MultiArray, '/joint3_velocity_controller/commands', 10)

        # Suscriptor de estados (para medir el Torque / Effort de Gazebo)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Definición de la Recta Cartesiana (P0 -> P1) en mm
        self.P0 = np.array([200.0, 50.0, 140.0])   # (X0, Y0, Z0)
        self.P1 = np.array([100.0, 200.0, 80.0])   # (X1, Y1, Z1)
        self.T_total = 5.0                         # Tiempo total del movimiento (segundos)
        self.dt = 0.02                             # Intervalo de tiempo (50 Hz)

        # Variables de control e historial
        self.t_actual = 0.0
        self.prev_q = None
        self.torques_actuales = [0.0, 0.0, 0.0]    # [Joint_1, Joint_2, Joint_3]
        self.datos_log = []
        self.finalizado = False

        # Timer de ejecución
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Iniciando generación de trayectoria recta...")

    def joint_state_cb(self, msg):
        # Lectura de esfuerzos/torques reales reportados por el motor de físicas de Gazebo
        try:
            if 'Joint_1' in msg.name and 'Joint_2' in msg.name and 'Joint_3' in msg.name:
                idx1 = msg.name.index('Joint_1')
                idx2 = msg.name.index('Joint_2')
                idx3 = msg.name.index('Joint_3')
                
                # Se asignan los torques (si la simulación los publica)
                t1 = msg.effort[idx1] if len(msg.effort) > idx1 else 0.0
                t2 = msg.effort[idx2] if len(msg.effort) > idx2 else 0.0
                t3 = msg.effort[idx3] if len(msg.effort) > idx3 else 0.0
                self.torques_actuales = [t1, t2, t3]
        except Exception as e:
            pass

    def calcular_ik(self, x, y, z):
        # 1. Eje Z (Prismático)
        d3_mm = H_BASE - z
        d3_m = d3_mm / 1000.0

        # 2. Codo Q2
        r2 = x**2 + y**2
        cos_q2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q2 = np.clip(cos_q2, -1.0, 1.0)
        sin_q2 = np.sqrt(1.0 - cos_q2**2)
        q2 = np.arctan2(sin_q2, cos_q2)

        # 3. Base Q1
        k1 = L1 + L2 * np.cos(q2)
        k2 = L2 * np.sin(q2)
        q1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return np.array([q1, q2, d3_m])

    def control_loop(self):
        if self.finalizado:
            return

        if self.t_actual <= self.T_total:
            # Ecuación paramétrica de la recta: P(t) = P0 + (t/T)*(P1 - P0)
            alpha = self.t_actual / self.T_total
            P_actual = self.P0 + alpha * (self.P1 - self.P0)
            x, y, z = P_actual

            # Posición articular actual (Cinemática Inversa)
            q_actual = self.calcular_ik(x, y, z)

            # Cálculo de Velocidades por Diferenciación Finita: v = (q_k - q_{k-1}) / dt
            if self.prev_q is None:
                v_articular = np.array([0.0, 0.0, 0.0])
            else:
                v_articular = (q_actual - self.prev_q) / self.dt

            # Enviar comandos a Gazebo
            msg1 = Float64MultiArray(); msg1.data = [float(q_actual[0])]
            msg2 = Float64MultiArray(); msg2.data = [float(q_actual[1])]
            msg3 = Float64MultiArray(); msg3.data = [float(v_articular[2])] # Velocidad para Joint_3

            self.pub_q1.publish(msg1)
            self.pub_q2.publish(msg2)
            self.pub_q3.publish(msg3)

            # Registrar datos en memoria: [Tiempo, X, Y, Z, q1, q2, d3, v1, v2, v3, Torque1, Torque2, Fuerza3]
            fila = [
                round(self.t_actual, 3),
                round(x, 2), round(y, 2), round(z, 2),
                round(q_actual[0], 4), round(q_actual[1], 4), round(q_actual[2], 4),
                round(v_articular[0], 4), round(v_articular[1], 4), round(v_articular[2], 4),
                round(self.torques_actuales[0], 4),
                round(self.torques_actuales[1], 4),
                round(self.torques_actuales[2], 4)
            ]
            self.datos_log.append(fila)

            # Avanzar tiempo y guardar estado
            self.prev_q = q_actual
            self.t_actual += self.dt
        else:
            # Frenar Joint 3 al terminar
            msg3 = Float64MultiArray(); msg3.data = [0.0]
            self.pub_q3.publish(msg3)

            self.guardar_csv()
            self.finalizado = True
            self.get_logger().info("¡Trayectoria completada con éxito y CSV generado!")

    def guardar_csv(self):
        ruta_csv = os.path.expanduser('~/ros2_ws/trayectoria_datos.csv')
        encabezados = [
            'tiempo_s', 'X_mm', 'Y_mm', 'Z_mm',
            'q1_rad', 'q2_rad', 'd3_m',
            'v1_rad_s', 'v2_rad_s', 'v3_m_s',
            'torque_q1_Nm', 'torque_q2_Nm', 'fuerza_d3_N'
        ]
        
        with open(ruta_csv, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(encabezados)
            writer.writerows(self.datos_log)
            
        print(f"\n[OK] Datos guardados exitosamente en: {ruta_csv}\n")

def main(args=None):
    rclpy.init(args=args)
    node = TrayectoriaLinealNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
