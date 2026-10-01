#!/usr/bin/env python3
"""
GUI — VERIFICACIÓN DE CINEMÁTICA DIRECTA E INVERSA — ROBOT R5 (SCARA)
Robótica 2026-1 — UMNG

Qué hace:
  - Cinemática DIRECTA: das q1,q2,d3 -> calcula (x,y,z) del efector final.
  - Cinemática INVERSA: das (x,y) objetivo + eliges codo arriba/abajo ->
    calcula q1,q2 (y d3 aparte, ya que es independiente de x,y).
  - Botones de acceso directo a PS y PF (coordenadas locales de R5 ya
    calculadas a partir del plano de planta).
  - Al mandar un movimiento al robot real (Gazebo), la GUI se suscribe a
    /joint_states, aplica la CD sobre los ángulos que el robot reporta de
    vuelta, y compara ese resultado contra el objetivo pedido -> esto es
    la VALIDACIÓN real de CD+CI, no solo el cálculo teórico.

Requiere: ROS 2 (rclpy) funcionando y la simulación ya lanzada
(ros2 launch .../scara_magico.launch.py) antes de correr este script.
"""
import threading
import numpy as np
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ── parámetros del robot (mismos de siempre) ─────────────────────────────
L1, L2 = 161.0, 112.0          # mm
LIM_Q1 = (-100.0, 140.0)       # grados — límites reales (URDF ya corregido)
LIM_Q2 = (-130.0, 130.0)       # grados
LIM_D3 = (-70.5, 0.0)          # mm — rango del prismático según URDF

# Puntos objetivo en marco LOCAL de R5 (derivados del plano de planta:
# base R5 en (-17.0,-45.0) cm global; PS=(0,-35) cm; PF=(-37,-35) cm)
PRESETS = {
    'PS (sellado)':   (170.0, 100.0),
    'PF (estantería)': (-200.0, 100.0),
}


# ── cinemática (idéntica a la ya validada en control_scara.py) ──────────
def fk(q1_deg, q2_deg):
    q1, q2 = np.radians(q1_deg), np.radians(q2_deg)
    x = L1*np.cos(q1) + L2*np.cos(q1+q2)
    y = L1*np.sin(q1) + L2*np.sin(q1+q2)
    return x, y

def ik(x, y, codo='arriba'):
    c2 = np.clip((x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2), -1.0, 1.0)
    s2 = np.sqrt(1 - c2**2) if codo == 'arriba' else -np.sqrt(1 - c2**2)
    q2 = np.degrees(np.arctan2(s2, c2))
    q1 = np.degrees(np.arctan2(y, x) - np.arctan2(L2*s2, L1 + L2*c2))
    return q1, q2

def dentro_limites(q1, q2, d3=0.0):
    return (LIM_Q1[0] <= q1 <= LIM_Q1[1] and
            LIM_Q2[0] <= q2 <= LIM_Q2[1] and
            LIM_D3[0] <= d3 <= LIM_D3[1])


# ── nodo ROS 2: publica trayectorias, escucha joint_states ──────────────
class NodoScara(Node):
    def __init__(self, callback_estado):
        super().__init__('gui_verificacion_cinematica')
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', callback_estado, 10)

    def mover(self, q1_deg, q2_deg, d3_mm, t_s=2.0):
        msg = JointTrajectory()
        msg.joint_names = ['Join_1', 'Joint_2', 'Joint_3']
        pt = JointTrajectoryPoint()
        pt.positions = [np.radians(q1_deg), np.radians(q2_deg), d3_mm/1000.0]
        pt.time_from_start = Duration(sec=int(t_s), nanosec=0)
        msg.points.append(pt)
        self.pub.publish(msg)


# ── GUI ────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root, nodo: NodoScara):
        self.root = root
        self.nodo = nodo
        self.objetivo_pedido = None   # (x,y) que se mandó, para comparar al validar

        root.title("Verificación CD / CI — R5 (SCARA)")
        root.geometry("620x640")

        nb = ttk.Notebook(root)
        nb.pack(fill='both', expand=True, padx=8, pady=8)

        # ── PESTAÑA 1: CINEMÁTICA DIRECTA ────────────────────────────────
        f_cd = ttk.Frame(nb); nb.add(f_cd, text="Cinemática Directa")
        ttk.Label(f_cd, text="Dado q1, q2, d3 -> calcular (x, y, z)",
                  font=('', 11, 'bold')).pack(pady=8)

        frm = ttk.Frame(f_cd); frm.pack(pady=4)
        self.cd_q1 = tk.DoubleVar(value=0.0)
        self.cd_q2 = tk.DoubleVar(value=0.0)
        self.cd_d3 = tk.DoubleVar(value=0.0)
        for i, (lbl, var) in enumerate([("q1 (°)", self.cd_q1),
                                         ("q2 (°)", self.cd_q2),
                                         ("d3 (mm)", self.cd_d3)]):
            ttk.Label(frm, text=lbl).grid(row=i, column=0, sticky='e', padx=4, pady=3)
            ttk.Entry(frm, textvariable=var, width=12).grid(row=i, column=1, padx=4)

        ttk.Button(f_cd, text="Calcular CD", command=self._calcular_cd).pack(pady=8)
        self.cd_resultado = tk.StringVar(value="x = —   y = —")
        ttk.Label(f_cd, textvariable=self.cd_resultado,
                  font=('Consolas', 11)).pack(pady=6)
        ttk.Button(f_cd, text="Mandar este q1,q2,d3 al robot (Gazebo)",
                   command=self._mandar_cd).pack(pady=10)

        # ── PESTAÑA 2: CINEMÁTICA INVERSA ────────────────────────────────
        f_ci = ttk.Frame(nb); nb.add(f_ci, text="Cinemática Inversa")
        ttk.Label(f_ci, text="Dado (x, y) objetivo -> calcular q1, q2",
                  font=('', 11, 'bold')).pack(pady=8)

        frm2 = ttk.Frame(f_ci); frm2.pack(pady=4)
        self.ci_x = tk.DoubleVar(value=170.0)
        self.ci_y = tk.DoubleVar(value=100.0)
        self.ci_d3 = tk.DoubleVar(value=0.0)
        for i, (lbl, var) in enumerate([("x (mm)", self.ci_x),
                                         ("y (mm)", self.ci_y),
                                         ("d3 (mm)", self.ci_d3)]):
            ttk.Label(frm2, text=lbl).grid(row=i, column=0, sticky='e', padx=4, pady=3)
            ttk.Entry(frm2, textvariable=var, width=12).grid(row=i, column=1, padx=4)

        self.ci_codo = tk.StringVar(value='arriba')
        fr_codo = ttk.Frame(f_ci); fr_codo.pack(pady=6)
        ttk.Radiobutton(fr_codo, text="Codo arriba", variable=self.ci_codo,
                         value='arriba').pack(side='left', padx=8)
        ttk.Radiobutton(fr_codo, text="Codo abajo", variable=self.ci_codo,
                         value='abajo').pack(side='left', padx=8)

        fr_presets = ttk.Frame(f_ci); fr_presets.pack(pady=6)
        for nombre, (px, py) in PRESETS.items():
            ttk.Button(fr_presets, text=nombre,
                       command=lambda x=px, y=py: (self.ci_x.set(x), self.ci_y.set(y))
                       ).pack(side='left', padx=4)

        ttk.Button(f_ci, text="Calcular CI", command=self._calcular_ci).pack(pady=8)
        self.ci_resultado = tk.StringVar(value="q1 = —   q2 = —")
        ttk.Label(f_ci, textvariable=self.ci_resultado,
                  font=('Consolas', 11)).pack(pady=4)
        self.ci_limite = tk.StringVar(value="")
        ttk.Label(f_ci, textvariable=self.ci_limite, foreground='#b00').pack()

        ttk.Button(f_ci, text="Enviar al robot (Gazebo)",
                   command=self._mandar_ci).pack(pady=10)

        # ── PESTAÑA 3: VALIDACIÓN EN VIVO ────────────────────────────────
        f_val = ttk.Frame(nb); nb.add(f_val, text="Validación en vivo")
        ttk.Label(f_val, text="Ángulos reales reportados por el robot (/joint_states)\n"
                              "y su verificación aplicando la CD sobre ellos",
                  font=('', 10, 'bold'), justify='center').pack(pady=8)

        self.val_texto = tk.Text(f_val, height=16, width=68, font=('Consolas', 9))
        self.val_texto.pack(pady=6, padx=6)
        self.val_texto.insert('end', "Esperando datos de /joint_states...\n")
        self.val_texto.config(state='disabled')

    # ── acciones pestaña CD ──────────────────────────────────────────────
    def _calcular_cd(self):
        x, y = fk(self.cd_q1.get(), self.cd_q2.get())
        z = self.cd_d3.get()   # z relativo (el offset fijo de la base no cambia la validación)
        self.cd_resultado.set(f"x = {x:8.2f} mm    y = {y:8.2f} mm    z(d3) = {z:6.2f} mm")

    def _mandar_cd(self):
        q1, q2, d3 = self.cd_q1.get(), self.cd_q2.get(), self.cd_d3.get()
        self.objetivo_pedido = fk(q1, q2)
        self.nodo.mover(q1, q2, d3)

    # ── acciones pestaña CI ──────────────────────────────────────────────
    def _calcular_ci(self):
        x, y, codo = self.ci_x.get(), self.ci_y.get(), self.ci_codo.get()
        q1, q2 = ik(x, y, codo)
        self.ci_resultado.set(f"q1 = {q1:8.3f}°    q2 = {q2:8.3f}°")
        if dentro_limites(q1, q2, self.ci_d3.get()):
            self.ci_limite.set("")
        else:
            self.ci_limite.set("⚠ FUERA de los límites articulares del robot")

    def _mandar_ci(self):
        x, y, codo = self.ci_x.get(), self.ci_y.get(), self.ci_codo.get()
        q1, q2 = ik(x, y, codo)
        if not dentro_limites(q1, q2, self.ci_d3.get()):
            self.ci_limite.set("⚠ FUERA de límites — no se envía")
            return
        self.objetivo_pedido = (x, y)
        self.nodo.mover(q1, q2, self.ci_d3.get())

    # ── callback de /joint_states (llamado desde el hilo de ROS 2) ──────
    def callback_joint_states(self, msg: JointState):
        try:
            idx = {n: i for i, n in enumerate(msg.name)}
            q1 = np.degrees(msg.position[idx['Join_1']])
            q2 = np.degrees(msg.position[idx['Joint_2']])
            d3 = msg.position[idx['Joint_3']] * 1000.0
        except (KeyError, IndexError):
            return

        x_fk, y_fk = fk(q1, q2)

        linea = f"q1={q1:8.3f}°  q2={q2:8.3f}°  d3={d3:7.2f}mm  ->  FK: x={x_fk:8.2f} y={y_fk:8.2f}"
        if self.objetivo_pedido is not None:
            xo, yo = self.objetivo_pedido
            err = np.hypot(x_fk - xo, y_fk - yo)
            linea += f"   |  objetivo=({xo:.1f},{yo:.1f})  error={err:.3f} mm"

        # actualizar el Text widget desde el hilo principal de Tkinter
        self.root.after(0, self._actualizar_texto, linea)

    def _actualizar_texto(self, linea):
        self.val_texto.config(state='normal')
        self.val_texto.insert('end', linea + '\n')
        self.val_texto.see('end')
        # limitar historial visible
        if int(self.val_texto.index('end-1c').split('.')[0]) > 200:
            self.val_texto.delete('1.0', '2.0')
        self.val_texto.config(state='disabled')


# ── main ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    rclpy.init()

    root = tk.Tk()
    app_ref = {}   # para poder pasar app.callback_joint_states al nodo antes de crear app

    def callback_puente(msg):
        if 'app' in app_ref:
            app_ref['app'].callback_joint_states(msg)

    nodo = NodoScara(callback_puente)
    app = App(root, nodo)
    app_ref['app'] = app

    threading.Thread(target=lambda: rclpy.spin(nodo), daemon=True).start()

    try:
        root.mainloop()
    finally:
        rclpy.shutdown()
