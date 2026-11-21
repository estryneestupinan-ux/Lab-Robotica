#!/usr/bin/env python3
"""
Secuencia de colocación de tapas con pinza (Guía 4), SCARA R4.

Ventana con el botón "Iniciar Secuencia" y controles manuales de la pinza.
La secuencia recorre, en el espacio articular:

  0. Abrir la pinza
  1. Home  ->  2. Sobre las tapas  ->  3. Bajar y CERRAR la pinza  ->  4. Subir con la tapa
  5. Sobre la botella  ->  6. Bajar y ABRIR la pinza  ->  7. Subir  ->  8. Volver a Home

Cada tramo usa un polinomio smoothstep, q(t) = q0 + (qf − q0)(3τ² − 2τ³), que arranca
y frena con velocidad cero. La duración de cada tramo se alarga sola si alguna junta
superaría VEL_MAX (la velocidad pico del smoothstep es 1.5·Δq/T).

Publica:  /joint1_position_controller/commands, /joint2_position_controller/commands,
          /joint3_target (lo ejecuta joint3_position_hold.py) y /pinza (grados; −1 = soltar).
Escucha:  /joint_states (Gazebo) y /real/joint_states (robot real, de puente_serial.py).

Con el robot real conectado, al final de cada tramo espera a que el robot llegue
(tolerancias TOLERANCIA, máximo T_MAX_ESPERA). Sin robot real, solo mueve Gazebo.
Después de abrir o cerrar, deja el servo sin pulsos (SOLTAR_PINZA): mientras el
servo recibe pulsos, los encoders del robot real dejan de contar.

Uso:  python3 secuencia_gui.py   (con Gazebo y joint3_position_hold.py corriendo;
                                  puente_serial.py si se usa el robot real)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
import tkinter as tk
import threading
import time
import math

HZ = 30  # pasos de interpolación por segundo

# --- Rango de J1 (base), en rad ---
# Para cada objetivo se usa el equivalente (objetivo ± 2π) que quede dentro de este rango
# y más cerca de la posición actual. Así la base nunca da una vuelta completa (los cables
# del robot real se enrollarían). -3.0 deja fuera a -π, para que Home sea siempre +π.
J1_MIN = -3.00
J1_MAX = 3.20

# --- Eje Z (Joint_3), en metros: 0 = arriba, positivo hacia abajo (d3 = H_BASE - z) ---
Z_ARRIBA = 0.000
Z_ABAJO = 0.030      # bajada para agarrar la tapa
Z_BOTELLA = Z_ABAJO  # bajada sobre la botella para sellar. Si la boca de la botella queda más
                     # alta que donde agarran la tapa, pongan aquí un número MENOR que Z_ABAJO.

# --- Pinza (servo), en grados. Búsquenlos con el slider de la ventana y cámbienlos aquí ---
PINZA_ABIERTA = 105
PINZA_CERRADA = 60
SERVO_GRADOS_S = 90.0   # velocidad del servo en el firmware (para saber cuánto esperar)
T_EXTRA_PINZA = 0.8     # espera adicional después de mover la pinza (el servo va con retraso)
T_ASENTAR = 1.0         # espera antes de mover la pinza: el Z real termina sus últimos milímetros
# Soltar el servo apenas llega (SERVO:OFF). Mientras el servo recibe pulsos, los encoders del
# robot no cuentan bien; suelto, la pinza igual sostiene la tapa (comprobado).
SOLTAR_PINZA = True

# --- Velocidad máxima por articulación (rad/s, rad/s, m/s) ---
# Base y codo: lo que da el robot real con PWM 120 (medido en la PRUEBA), con margen.
# El codo va por correa 20:60, así que gira a un tercio del motor (~0.16 rad/s máximo).
# Z: el N20 de 30 RPM con husillo T8x8 da como máximo ~4 mm/s (confirmado midiendo:
#    10 mm pedidos = 10 mm reales). Se usa 2.8 mm/s de pico, por debajo del límite del puente (3 mm/s).
# Si un tramo pide más, su duración se alarga automáticamente.
VEL_MAX = (0.35, 0.11, 0.0028)
FACTOR_PICO = 1.5  # el smoothstep tiene velocidad pico = 1.5 * distancia / duración

# --- Esperar al robot real ---
# Si puente_serial.py está corriendo, al final de cada paso la secuencia espera a que el
# robot real llegue antes de seguir (así no baja el Z mientras la base todavía gira).
# Sin robot real (solo Gazebo) no espera. La tolerancia del Z es 3 mm porque
# joint3_position_hold.py detiene el Z de Gazebo a menos de 2 mm del objetivo.
ESPERAR_REAL = ("Joint_1", "Joint_2", "Joint_3")
TOLERANCIA = {"Joint_1": 0.02, "Joint_2": 0.02, "Joint_3": 0.003}
T_MAX_ESPERA = 60.0
NOMBRES = ("Joint_1", "Joint_2", "Joint_3")


def objetivo_j1(actual, objetivo):
    """Equivalente de 'objetivo' (+/- 2*pi) dentro de [J1_MIN, J1_MAX] más cercano a 'actual'."""
    candidatos = [objetivo + 2 * math.pi * k for k in range(-2, 3)]
    validos = [c for c in candidatos if J1_MIN <= c <= J1_MAX]
    if not validos:
        raise ValueError(f"J1 = {objetivo:.3f} rad no cabe en [{J1_MIN}, {J1_MAX}]")
    return min(validos, key=lambda c: abs(c - actual))


def suavizar(t):
    """Smoothstep: arranca y frena suave (t entre 0 y 1)."""
    return t * t * (3 - 2 * t)


class SecuenciaNode(Node):
    def __init__(self):
        super().__init__('secuencia_gui_node')
        self.pub1 = self.create_publisher(Float64MultiArray, "/joint1_position_controller/commands", 10)
        self.pub2 = self.create_publisher(Float64MultiArray, "/joint2_position_controller/commands", 10)
        self.pub3_target = self.create_publisher(Float64, "/joint3_target", 10)
        self.pub_pinza = self.create_publisher(Float64, "/pinza", 10)
        self.pinza_actual = None   # último ángulo mandado (None = no se sabe dónde está)
        self._timer_soltar = None

        self.lock = threading.Lock()
        self.gazebo = {}            # última posición de Gazebo
        self.real = {}              # última posición del robot real
        self.t_real = -1e9
        self.create_subscription(JointState, "/joint_states", self._de_gazebo, 10)
        self.create_subscription(JointState, "/real/joint_states", self._de_real, 10)

        self.current_j1 = 3.142
        self.current_j2 = 1.800
        self.current_j3 = Z_ARRIBA

    def _de_gazebo(self, msg):
        with self.lock:
            self.gazebo.update(zip(msg.name, msg.position))

    def _de_real(self, msg):
        with self.lock:
            self.real.update(zip(msg.name, msg.position))
            self.t_real = time.monotonic()

    def tomar_pose_de_gazebo(self):
        """Arranca la interpolación desde donde está Gazebo de verdad (no desde un supuesto)."""
        with self.lock:
            gz = dict(self.gazebo)
        if all(n in gz for n in NOMBRES):
            self.current_j1, self.current_j2, self.current_j3 = (gz[n] for n in NOMBRES)

    def publish_joints(self, j1, j2, j3):
        msg1 = Float64MultiArray(); msg1.data = [float(j1)]
        self.pub1.publish(msg1)

        msg2 = Float64MultiArray(); msg2.data = [float(j2)]
        self.pub2.publish(msg2)

        msg3 = Float64(); msg3.data = float(j3)
        self.pub3_target.publish(msg3)

    def soltar_pinza(self):
        """Deja el servo sin pulsos (SERVO:OFF): queda donde está y no molesta a los encoders."""
        msg = Float64(); msg.data = -1.0
        self.pub_pinza.publish(msg)

    def mover_pinza(self, angulo, esperar=True):
        """Manda el ángulo a la pinza (puente_serial.py lo pasa al ESP32), espera a que llegue y la suelta."""
        if self._timer_soltar is not None:
            self._timer_soltar.cancel()
            self._timer_soltar = None
        msg = Float64(); msg.data = float(angulo)
        self.pub_pinza.publish(msg)
        anterior, self.pinza_actual = self.pinza_actual, float(angulo)
        recorrido = 180.0 if anterior is None else abs(angulo - anterior)
        t_llegar = recorrido / SERVO_GRADOS_S + T_EXTRA_PINZA
        if esperar:
            time.sleep(t_llegar)
            if SOLTAR_PINZA:
                self.soltar_pinza()
        elif SOLTAR_PINZA:
            # Botones y slider: se suelta cuando llegue, sin congelar la ventana
            self._timer_soltar = threading.Timer(t_llegar, self.soltar_pinza)
            self._timer_soltar.daemon = True
            self._timer_soltar.start()

    def mover_suave(self, target_j1, target_j2, target_j3, duracion=5.0):
        target_j1 = objetivo_j1(self.current_j1, target_j1)

        inicio = (self.current_j1, self.current_j2, self.current_j3)
        fin = (target_j1, target_j2, target_j3)

        # Alargar el tramo si alguna articulación pasaría su velocidad máxima
        dur_min = max(FACTOR_PICO * abs(b - a) / v for a, b, v in zip(inicio, fin, VEL_MAX))
        if dur_min > duracion:
            self.get_logger().warning(
                f"Tramo alargado de {duracion:.1f} s a {dur_min:.1f} s para respetar VEL_MAX")
            duracion = dur_min

        pasos = max(100, int(duracion * HZ))
        espera = duracion / pasos
        t0 = time.monotonic()

        for i in range(1, pasos + 1):
            s = suavizar(i / pasos)
            self.current_j1 = inicio[0] + (fin[0] - inicio[0]) * s
            self.current_j2 = inicio[1] + (fin[1] - inicio[1]) * s
            self.current_j3 = inicio[2] + (fin[2] - inicio[2]) * s
            self.publish_joints(self.current_j1, self.current_j2, self.current_j3)
            # Reloj absoluto: evita que el retraso de cada sleep se acumule
            restante = t0 + i * espera - time.monotonic()
            if restante > 0:
                time.sleep(restante)

        # Posición exacta al final
        self.current_j1, self.current_j2, self.current_j3 = fin
        self.publish_joints(self.current_j1, self.current_j2, self.current_j3)
        return dict(zip(NOMBRES, fin))

    def esperar_robot_real(self, objetivo):
        """Espera a que el robot real llegue a 'objetivo'. Sin robot real conectado, no espera."""
        t0 = time.monotonic()
        while True:
            with self.lock:
                hay_real = time.monotonic() - self.t_real < 1.0
                real = dict(self.real)
            if not hay_real:
                return True   # solo simulación
            faltan = [n for n in ESPERAR_REAL
                      if n in real and abs(real[n] - objetivo[n]) > TOLERANCIA[n]]
            if not faltan:
                return True
            if time.monotonic() - t0 > T_MAX_ESPERA:
                self.get_logger().error(f"El robot real no llegó en {T_MAX_ESPERA:.0f} s: "
                                        + ", ".join(f"{n} real={real[n]:.3f} objetivo={objetivo[n]:.3f}"
                                                    for n in faltan))
                return False
            time.sleep(0.05)


def ejecutar_secuencia(node, lbl_estado, btn_iniciar):
    btn_iniciar.config(state=tk.DISABLED)
    node.tomar_pose_de_gazebo()

    lbl_estado.config(text="Abriendo la pinza...")
    node.mover_pinza(PINZA_ABIERTA)

    # (nombre, j1, j2, j3, duracion_s, pausa_al_llegar_s, pinza al llegar)
    secuencia = [
        ("1. Home",              3.142,  1.800, Z_ARRIBA,  4.0, 0.5, None),
        ("2. Ir a tapas",       -1.443,  0.918, Z_ARRIBA,  5.0, 0.5, None),
        ("3. Bajar por tapa",   -1.443,  0.918, Z_ABAJO,   6.0, 0.5, PINZA_CERRADA),
        ("4. Subir con tapa",   -1.443,  0.918, Z_ARRIBA,  6.0, 0.5, None),
        ("5. Ir a botella",      0.391,  0.866, Z_ARRIBA,  5.0, 0.5, None),
        ("6. Bajar para sellar", 0.391,  0.866, Z_BOTELLA, 6.0, 0.5, PINZA_ABIERTA),
        ("7. Subir (liberar)",   0.391,  0.866, Z_ARRIBA,  6.0, 0.5, None),
        ("8. Volver a Home",     3.142,  1.800, Z_ARRIBA,  5.0, 0.5, None),
    ]

    for paso, j1, j2, j3, dur, pausa, pinza in secuencia:
        lbl_estado.config(text=f"Ejecutando: {paso}")
        objetivo = node.mover_suave(j1, j2, j3, duracion=dur)
        lbl_estado.config(text=f"{paso}: esperando al robot real...")
        if not node.esperar_robot_real(objetivo):
            lbl_estado.config(text=f"DETENIDA en {paso}: el robot real no llegó")
            btn_iniciar.config(state=tk.NORMAL)
            return
        if pinza is not None:
            accion = "cerrando" if pinza == PINZA_CERRADA else "abriendo"
            time.sleep(T_ASENTAR)
            lbl_estado.config(text=f"{paso}: {accion} la pinza ({pinza:.0f} grados)...")
            node.mover_pinza(pinza)
        time.sleep(pausa)

    lbl_estado.config(text="Estado: Secuencia completada.")
    btn_iniciar.config(state=tk.NORMAL)


def iniciar_gui(node):
    root = tk.Tk()
    root.title("Secuencia SCARA R4")
    root.geometry("440x250")

    lbl_estado = tk.Label(root, text="Estado: Esperando inicio...", font=("Arial", 10))
    lbl_estado.pack(pady=10)

    # --- Pinza a mano: para encontrar los ángulos sin cerrar nada ---
    marco = tk.LabelFrame(root, text="Pinza (grados del servo)", padx=8, pady=4)
    marco.pack(fill="x", padx=12)
    deslizador = tk.Scale(marco, from_=0, to=180, orient=tk.HORIZONTAL, length=380)
    deslizador.set(PINZA_ABIERTA)
    deslizador.pack()
    # Se manda al soltar el slider (no en cada grado mientras se arrastra)
    deslizador.bind("<ButtonRelease-1>", lambda _e: node.mover_pinza(deslizador.get(), esperar=False))
    botones = tk.Frame(marco)
    botones.pack()

    def ir(angulo):
        deslizador.set(angulo)
        node.mover_pinza(angulo, esperar=False)

    tk.Button(botones, text=f"Abrir ({PINZA_ABIERTA})", width=14,
              command=lambda: ir(PINZA_ABIERTA)).pack(side=tk.LEFT, padx=4)
    tk.Button(botones, text=f"Cerrar ({PINZA_CERRADA})", width=14,
              command=lambda: ir(PINZA_CERRADA)).pack(side=tk.LEFT, padx=4)

    def on_iniciar_click():
        hilo = threading.Thread(target=ejecutar_secuencia, args=(node, lbl_estado, btn_iniciar), daemon=True)
        hilo.start()

    btn_iniciar = tk.Button(
        root,
        text="Iniciar Secuencia",
        command=on_iniciar_click,
        font=("Arial", 12, "bold"),
        bg="green",
        fg="white"
    )
    btn_iniciar.pack(pady=14)

    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    nodo_secuencia = SecuenciaNode()

    hilo_gui = threading.Thread(target=iniciar_gui, args=(nodo_secuencia,), daemon=True)
    hilo_gui.start()

    try:
        rclpy.spin(nodo_secuencia)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            nodo_secuencia.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
