#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import threading

# Parámetros CAD Reales (mm)
L1 = 120.0
L2 = 160.0
H_BASE = 163.0
D3_MAX = 105.0

# Parámetros del control de Joint_3 (eje Z, prismático)
KP_Z = 3.0
V_MAX_Z = 0.1          # límite de velocidad del URDF (m/s)
MAX_ACCEL_Z = 0.15     # rampa de aceleración (m/s^2) -- esto es lo que evita el bug de gz-sim
POS_TOLERANCE_Z = 0.0015

CONTROL_PERIOD = 0.033  # 30 Hz


class IKGuiNode(Node):
    def __init__(self):
        super().__init__('ik_gui_teleop')

        self.pub_q1 = self.create_publisher(Float64MultiArray, '/joint1_position_controller/commands', 10)
        self.pub_q2 = self.create_publisher(Float64MultiArray, '/joint2_position_controller/commands', 10)
        self.pub_q3 = self.create_publisher(Float64MultiArray, '/joint3_velocity_controller/commands', 10)

        # Suscriptor para leer la posición real desde Gazebo
        self.sub_states = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Metas finales que pide la GUI
        self.target_q1 = np.arctan2(0.0, 280.0)  # Home X=280, Y=0
        self.target_q2 = 0.0
        self.target_d3 = 0.0

        # Posiciones actuales para interpolación suave
        self.current_q1 = self.target_q1
        self.current_q2 = 0.0

        # Estado real de Gazebo
        self.real_d3 = 0.0
        self.gazebo_connected = False

        # Velocidad de Joint_3 actualmente comandada (para la rampa de aceleración)
        self.cmd_vel3 = 0.0

        # Timer (Bucle de control a 30Hz)
        self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.get_logger().info("Nodo IK listo. Controlador Activo con rampa de aceleracion en Z.")

    def joint_state_cb(self, msg):
        try:
            idx = msg.name.index('Joint_3')
            self.real_d3 = msg.position[idx]
            self.gazebo_connected = True
        except ValueError:
            pass

    def control_loop(self):
        # 1. Movimiento Suave para Q1 y Q2 (sin cambios, esto ya funcionaba bien)
        step = 0.02  # Velocidad de giro por ciclo (Radianes)

        if abs(self.target_q1 - self.current_q1) > step:
            self.current_q1 += step * np.sign(self.target_q1 - self.current_q1)
        else:
            self.current_q1 = self.target_q1

        if abs(self.target_q2 - self.current_q2) > step:
            self.current_q2 += step * np.sign(self.target_q2 - self.current_q2)
        else:
            self.current_q2 = self.target_q2

        msg1 = Float64MultiArray(); msg1.data = [float(self.current_q1)]
        msg2 = Float64MultiArray(); msg2.data = [float(self.current_q2)]
        self.pub_q1.publish(msg1)
        self.pub_q2.publish(msg2)

        # 2. Control de Joint_3 (Z) con rampa de aceleracion
        if self.gazebo_connected:
            error = self.target_d3 - self.real_d3

            if abs(error) < POS_TOLERANCE_Z:
                desired_vel = 0.0
            else:
                desired_vel = KP_Z * error
                desired_vel = max(-V_MAX_Z, min(V_MAX_Z, desired_vel))

            # Nunca dejar que la velocidad comandada cambie de golpe:
            # esto es lo que evita disparar el bug de gz-sim (#2785)
            max_step = MAX_ACCEL_Z * CONTROL_PERIOD
            delta = desired_vel - self.cmd_vel3
            delta = max(-max_step, min(max_step, delta))
            self.cmd_vel3 += delta

            msg3 = Float64MultiArray(); msg3.data = [float(self.cmd_vel3)]
            self.pub_q3.publish(msg3)

    def send_ik_command(self, x_mm, y_mm, z_mm):
        d3 = H_BASE - z_mm
        if d3 < 0.0 or d3 > D3_MAX:
            z_min = H_BASE - D3_MAX
            return False, f"Z = {z_mm} mm fuera de rango. Debe estar entre {z_min:.0f} y {H_BASE:.0f} mm."

        r2 = x_mm**2 + y_mm**2
        cos_q2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

        if abs(cos_q2) > 1.0:
            return False, f"Punto ({x_mm}, {y_mm}) inalcanzable."

        sin_q2 = np.sqrt(1.0 - cos_q2**2)
        q2_rad = np.arctan2(sin_q2, cos_q2)

        k1 = L1 + L2 * np.cos(q2_rad)
        k2 = L2 * np.sin(q2_rad)
        q1_rad = np.arctan2(y_mm, x_mm) - np.arctan2(k2, k1)

        # Actualizar las metas (El Timer hara el resto del trabajo)
        self.target_q1 = q1_rad
        self.target_q2 = q2_rad
        self.target_d3 = d3 / 1000.0

        info_txt = (f"Enviando metas:\n"
                    f"q1 = {np.degrees(q1_rad):.2f}°\n"
                    f"q2 = {np.degrees(q2_rad):.2f}°\n"
                    f"d3 = {d3:.2f} mm")
        return True, info_txt


def launch_gui(ros_node):
    root = tk.Tk()
    root.title("Control IK - Robot SCARA")
    root.geometry("400x500")

    frame = ttk.Frame(root, padding="15")
    frame.pack(fill=tk.BOTH, expand=True)

    ttk.Label(frame, text="CINEMÁTICA INVERSA (XYZ -> Gazebo)", font=('Helvetica', 10, 'bold')).pack(pady=5)

    fields_frame = ttk.Frame(frame)
    fields_frame.pack(pady=10)

    ttk.Label(fields_frame, text="X (mm):").grid(row=0, column=0, padx=5, pady=5)
    entry_x = ttk.Entry(fields_frame, width=12)
    entry_x.grid(row=0, column=1)
    entry_x.insert(0, "280.0")

    ttk.Label(fields_frame, text="Y (mm):").grid(row=1, column=0, padx=5, pady=5)
    entry_y = ttk.Entry(fields_frame, width=12)
    entry_y.grid(row=1, column=1)
    entry_y.insert(0, "0.0")

    ttk.Label(fields_frame, text="Z (mm):").grid(row=2, column=0, padx=5, pady=5)
    entry_z = ttk.Entry(fields_frame, width=12)
    entry_z.grid(row=2, column=1)
    entry_z.insert(0, "163.0")

    lbl_status = ttk.Label(frame, text="Estado: Listo", font=('Helvetica', 9, 'italic'), wraplength=320, foreground="blue")

    def on_send():
        try:
            x, y, z = float(entry_x.get()), float(entry_y.get()), float(entry_z.get())
            success, msg = ros_node.send_ik_command(x, y, z)
            lbl_status.config(text=msg, foreground="green" if success else "red")
        except ValueError:
            messagebox.showerror("Error", "Ingrese valores numéricos válidos.")

    def on_home():
        entry_x.delete(0, tk.END); entry_x.insert(0, "280.0")
        entry_y.delete(0, tk.END); entry_y.insert(0, "0.0")
        entry_z.delete(0, tk.END); entry_z.insert(0, "163.0")
        on_send()

    btn_send = ttk.Button(frame, text="Enviar Posición a Gazebo", command=on_send)
    btn_send.pack(fill=tk.X, pady=10)

    btn_home = ttk.Button(frame, text="🏠 Ir a HOME (Origen)", command=on_home)
    btn_home.pack(fill=tk.X, pady=5)

    lbl_status.pack(pady=10)

    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = IKGuiNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    launch_gui(node)


if __name__ == '__main__':
    main()
