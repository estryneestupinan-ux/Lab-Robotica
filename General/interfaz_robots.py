#!/usr/bin/env python3

import threading
import time
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

#  CONFIGURACIÓN (editable) 
NOMBRES_ROBOTS = ["robot1", "robot2", "robot3", "robot4", "robot5"]

TOPICO_INICIO = "/secuencia/inicio"          # std_msgs/Bool
TOPICO_EMERGENCIA = "/emergencia"            # std_msgs/Bool

MODO_LECTURA = "bool"                        # "bool" o "joint_states"
TOPICO_MOVIMIENTO = "/{robot}/en_movimiento" # usado si MODO_LECTURA == "bool"
TOPICO_JOINTS = "/{robot}/joint_states"      # usado si MODO_LECTURA == "joint_states"
UMBRAL_VELOCIDAD = 0.01                      # rad/s o m/s

TIEMPO_SIN_DATOS = 2.0      # s sin mensajes -> robot marcado "SIN DATOS"
PERIODO_EMERGENCIA = 0.1    # s entre re-publicaciones de la emergencia
PERIODO_GUI = 100           # ms entre refrescos de la interfaz
# ===


class NodoControl(Node):
    """Nodo ROS 2: publica inicio/emergencia y escucha el estado de los robots."""

    def __init__(self):
        super().__init__("interfaz_robots")

        # Inicio: confiable, sin persistencia (un robot que se reinicie NO
        # recibe un "inicio" viejo).
        qos_inicio = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Emergencia: confiable y persistente (un nodo que se conecte tarde
        # recibe el último estado de emergencia).
        qos_emergencia = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_inicio = self.create_publisher(Bool, TOPICO_INICIO, qos_inicio)
        self.pub_emergencia = self.create_publisher(Bool, TOPICO_EMERGENCIA, qos_emergencia)

        self._lock = threading.Lock()
        self.estado = {n: {"movimiento": None, "ultimo": 0.0} for n in NOMBRES_ROBOTS}
        self.emergencia_activa = False

        for nombre in NOMBRES_ROBOTS:
            if MODO_LECTURA == "bool":
                self.create_subscription(
                    Bool,
                    TOPICO_MOVIMIENTO.format(robot=nombre),
                    lambda msg, n=nombre: self._actualizar(n, bool(msg.data)),
                    10,
                )
            elif MODO_LECTURA == "joint_states":
                self.create_subscription(
                    JointState,
                    TOPICO_JOINTS.format(robot=nombre),
                    lambda msg, n=nombre: self._actualizar(
                        n, any(abs(v) > UMBRAL_VELOCIDAD for v in msg.velocity)
                    ),
                    10,
                )
            else:
                raise ValueError("MODO_LECTURA debe ser 'bool' o 'joint_states'")

        # Re-publica la emergencia mientras esté activa (por si se pierde un mensaje)
        self.create_timer(PERIODO_EMERGENCIA, self._reenviar_emergencia)

        # Estado inicial conocido: sin emergencia
        self.pub_emergencia.publish(Bool(data=False))
        self.get_logger().info("Interfaz de robots lista.")

    #  callbacks 
    def _actualizar(self, nombre, en_movimiento):
        with self._lock:
            self.estado[nombre]["movimiento"] = en_movimiento
            self.estado[nombre]["ultimo"] = time.monotonic()

    def _reenviar_emergencia(self):
        if self.emergencia_activa:
            self.pub_emergencia.publish(Bool(data=True))

    #  acciones desde la GUI 
    def iniciar_secuencia(self):
        if self.emergencia_activa:
            return False
        self.pub_inicio.publish(Bool(data=True))
        self.get_logger().info("Secuencia iniciada.")
        return True

    def activar_emergencia(self):
        self.emergencia_activa = True
        self.pub_emergencia.publish(Bool(data=True))
        self.get_logger().warn("PARADA DE EMERGENCIA ACTIVADA")

    def rearmar(self):
        self.emergencia_activa = False
        self.pub_emergencia.publish(Bool(data=False))
        self.get_logger().info("Emergencia liberada (rearme).")

    def leer_estados(self):
        """Devuelve {robot: True/False/None}; None = sin datos recientes."""
        ahora = time.monotonic()
        resultado = {}
        with self._lock:
            for n, e in self.estado.items():
                if e["movimiento"] is None or ahora - e["ultimo"] > TIEMPO_SIN_DATOS:
                    resultado[n] = None
                else:
                    resultado[n] = e["movimiento"]
        return resultado


class Interfaz:
    """Ventana Tkinter."""

    COLOR_MOV = "#2e7d32"
    COLOR_DET = "#9e9e9e"
    COLOR_SIN = "#f9a825"

    def __init__(self, root, nodo):
        self.root = root
        self.nodo = nodo
        root.title("Control de secuencia - 5 robots (ROS 2)")
        root.resizable(False, False)

        #  Estado de robots 
        marco_robots = tk.LabelFrame(root, text="Estado de los robots", padx=10, pady=10)
        marco_robots.grid(row=0, column=0, columnspan=3, padx=10, pady=10, sticky="ew")

        self.indicadores = {}
        for i, nombre in enumerate(NOMBRES_ROBOTS):
            tk.Label(marco_robots, text=nombre, font=("Arial", 11, "bold")).grid(
                row=0, column=i, padx=12)
            canvas = tk.Canvas(marco_robots, width=40, height=40, highlightthickness=0)
            circulo = canvas.create_oval(4, 4, 36, 36, fill=self.COLOR_SIN, outline="black")
            canvas.grid(row=1, column=i, pady=4)
            etiqueta = tk.Label(marco_robots, text="SIN DATOS", width=13)
            etiqueta.grid(row=2, column=i)
            self.indicadores[nombre] = (canvas, circulo, etiqueta)

        #  Botones 
        self.btn_inicio = tk.Button(root, text="Iniciar secuencia", font=("Arial", 12),
                                    width=18, height=2, command=self.on_inicio)
        self.btn_inicio.grid(row=1, column=0, padx=10, pady=5)

        self.btn_emergencia = tk.Button(root, text="PARADA DE\nEMERGENCIA",
                                        font=("Arial", 14, "bold"), bg="#c62828", fg="white",
                                        activebackground="#8e0000", activeforeground="white",
                                        width=16, height=3, command=self.on_emergencia)
        self.btn_emergencia.grid(row=1, column=1, padx=10, pady=5)

        self.btn_rearmar = tk.Button(root, text="Rearmar", font=("Arial", 12),
                                     width=18, height=2, state="disabled",
                                     command=self.on_rearmar)
        self.btn_rearmar.grid(row=1, column=2, padx=10, pady=5)

        #  Barra de estado 
        self.lbl_estado = tk.Label(root, text="Listo. (Esc o Espacio = emergencia)",
                                   anchor="w", relief="sunken")
        self.lbl_estado.grid(row=2, column=0, columnspan=3, sticky="ew", padx=10, pady=(5, 10))

        # Atajos de teclado para la emergencia
        root.bind("<Escape>", lambda e: self.on_emergencia())
        root.bind("<space>", lambda e: self.on_emergencia())

        root.protocol("WM_DELETE_WINDOW", self.on_cerrar)
        root.after(PERIODO_GUI, self.refrescar)

    #  eventos 
    def on_inicio(self):
        if self.nodo.iniciar_secuencia():
            self.lbl_estado.config(text=f"Secuencia iniciada - {time.strftime('%H:%M:%S')}")
        else:
            messagebox.showwarning("Emergencia activa",
                                   "Debe rearmar el sistema antes de iniciar.")

    def on_emergencia(self):
        self.nodo.activar_emergencia()
        self.btn_inicio.config(state="disabled")
        self.btn_rearmar.config(state="normal")
        self.lbl_estado.config(
            text=f"EMERGENCIA ACTIVA desde {time.strftime('%H:%M:%S')}", fg="#c62828")

    def on_rearmar(self):
        if not messagebox.askyesno("Rearmar",
                                   "¿Confirma que la zona es segura y desea liberar la emergencia?"):
            return
        self.nodo.rearmar()
        self.btn_inicio.config(state="normal")
        self.btn_rearmar.config(state="disabled")
        self.lbl_estado.config(text="Sistema rearmado. Listo para iniciar.", fg="black")

    def on_cerrar(self):
        if messagebox.askokcancel("Salir", "¿Cerrar la interfaz?"):
            self.root.destroy()

    #  refresco periódico 
    def refrescar(self):
        for nombre, valor in self.nodo.leer_estados().items():
            canvas, circulo, etiqueta = self.indicadores[nombre]
            if valor is None:
                canvas.itemconfig(circulo, fill=self.COLOR_SIN)
                etiqueta.config(text="SIN DATOS")
            elif valor:
                canvas.itemconfig(circulo, fill=self.COLOR_MOV)
                etiqueta.config(text="EN MOVIMIENTO")
            else:
                canvas.itemconfig(circulo, fill=self.COLOR_DET)
                etiqueta.config(text="DETENIDO")
        self.root.after(PERIODO_GUI, self.refrescar)


def main():
    rclpy.init()
    nodo = NodoControl()

    # ROS 2 gira en un hilo aparte; Tkinter debe quedarse en el hilo principal
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)
    hilo_ros = threading.Thread(target=ejecutor.spin, daemon=True)
    hilo_ros.start()

    root = tk.Tk()
    Interfaz(root, nodo)
    try:
        root.mainloop()
    finally:
        ejecutor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
