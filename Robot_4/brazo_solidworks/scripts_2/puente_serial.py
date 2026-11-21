#!/usr/bin/env python3
"""
Puente Gazebo -> robot real, SCARA R4.

El robot real COPIA al robot de Gazebo. Este nodo escucha /joint_states (lo que
publica Gazebo), limita la velocidad a lo que dan los motores reales y le manda al
ESP32 la posición de cada articulación en cuentas de encoder. Por eso todos sus
scripts (sliders, IK, secuencia, trayectoria lineal) mueven el robot real sin cambios.

Además publica la posición medida del robot real en /real/joint_states y guarda un
CSV con Gazebo vs. robot real en ~/ros2_ws/robot_real_<fecha>.csv
(para graficarlo: python3 graficar_comparacion.py).

ANTES DE CORRERLO
  1. El robot real tiene que estar en la pose cero de Gazebo: Joint_1 = 0 y
     Joint_2 = 0 (brazo estirado, como aparece en Gazebo al arrancar) y Joint_3 = 0
     (Z arriba). En esa pose mándenle ZERO al ESP32 desde el monitor serial.
  2. Cierren el monitor serial: el puerto lo puede usar un solo programa a la vez.

Pinza: escucha /pinza (std_msgs/Float64, ángulo del servo en grados, 0 a 180) y se lo
manda al ESP32. Un valor negativo suelta el servo (SERVO:OFF). Lo publica secuencia_gui.py (sus botones, su slider y la secuencia).

Protocolo con el ESP32 (115200 baudios, una línea por mensaje):
  PC -> ESP32 : SET:<c1>,<c2>,<c3>                  cuentas de M1 (Z), M2 (codo), M3 (base)
                SERVO:<grados>                      servo de la pinza
  ESP32 -> PC : POS:<t_ms>,<c1>,<c2>,<c3>,<pwm...>   telemetría cada 20 ms
"""
import csv
import math
import os
import threading
import time
from datetime import datetime

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# ============================ CONFIGURACIÓN ============================
PUERTO = "/dev/esp32"         # nombre fijo que crea la regla udev 99-esp32.rules
BAUDIOS = 115200
HZ_ENVIO = 50                # igual al Ts = 20 ms del ESP32

TOPICO_GAZEBO = "/joint_states"
TOPICO_REAL = "/real/joint_states"
TOPICO_PINZA = "/pinza"     # ángulo del servo de la pinza, en grados
ARTICULACIONES = ("Joint_1", "Joint_2", "Joint_3")   # nombres en el URDF

# Conector de la PCB de cada articulación: M1 = Z, M2 = codo (correa), M3 = base
MOTOR_DE = {"Joint_1": 3, "Joint_2": 2, "Joint_3": 1}

# +1 si las cuentas positivas mueven la articulación en el sentido positivo de Gazebo.
# Medido: con cuentas +, la base y el codo giran HORARIO (vistos desde arriba) y el Z BAJA.
# En Gazebo el ángulo positivo es ANTIHORARIO (eje "0 0 1") y Joint_3 positivo es hacia
# abajo (d3 = H_BASE - z). Se comprueba con los sliders: si el robot real gira al revés
# que Gazebo en una articulación, cambien el signo de esa articulación.
SIGNO = {"Joint_1": -1, "Joint_2": -1, "Joint_3": +1}

# Cuentas por unidad de Gazebo (rad en Joint_1 y Joint_2, METROS en Joint_3).
# El ESP32 cuenta solo los flancos de subida del canal A:
#   base: JGB37-520 12 RPM, 11 pulsos x 810:1 = 8910 cuentas/vuelta
#   codo: lo mismo, más la correa de 20 a 60 dientes (3:1)
#   Z:    N20 30 RPM, 7 pulsos x 1000:1 = 7000 cuentas/vuelta, husillo de 8 mm/vuelta
CUENTAS_RAD_MOTOR = 11 * 810 / (2 * math.pi)                # 1418.1
CUENTAS = {
    "Joint_1": CUENTAS_RAD_MOTOR,                           # 1418.1 cuentas/rad
    "Joint_2": CUENTAS_RAD_MOTOR * 60 / 20,                 # 4254.2 cuentas/rad (correa 20:60)
    "Joint_3": 7000 / 8.0 * 1000,                           # 875000 cuentas/m (875/mm)
}

# Pose de Gazebo que corresponde a las cuentas en 0 (donde se mandó ZERO)
HOME = {"Joint_1": 0.0, "Joint_2": 0.0, "Joint_3": 0.0}

# Velocidad máxima con la que el robot real sigue a Gazebo (rad/s, rad/s, m/s).
# Si Gazebo va más rápido, el robot real lo sigue a esta velocidad y lo alcanza después.
VEL_MAX = {"Joint_1": 0.40, "Joint_2": 0.12, "Joint_3": 0.003}

# Rango permitido en el robot real: lo que Gazebo pida fuera de aquí se recorta.
LIMITES = {"Joint_1": (-3.20, 3.20), "Joint_2": (-2.50, 2.50), "Joint_3": (0.0, 0.105)}

# El Z real se queda quieto arriba hasta confirmar cuántas cuentas por mm tiene.
MOVER_Z = True
# =======================================================================

assert sorted(MOTOR_DE.values()) == [1, 2, 3], "MOTOR_DE debe usar 1, 2 y 3 una vez cada uno"


def a_cuentas(nombre, valor):
    """Posición de Gazebo -> cuentas de encoder relativas al cero."""
    return int(round(SIGNO[nombre] * (valor - HOME[nombre]) * CUENTAS[nombre]))


def de_cuentas(nombre, cuentas):
    """Cuentas de encoder -> posición en unidades de Gazebo."""
    return HOME[nombre] + SIGNO[nombre] * cuentas / CUENTAS[nombre]


def procesos_usando(puerto):
    """Otros programas que tienen abierto el puerto (Linux), p. ej. el monitor serial."""
    try:
        real = os.path.realpath(puerto)
        pids = [p for p in os.listdir("/proc") if p.isdigit() and int(p) != os.getpid()]
    except OSError:
        return []
    otros = []
    for pid in pids:
        carpeta = f"/proc/{pid}/fd"
        try:
            for fd in os.listdir(carpeta):
                if os.path.realpath(os.path.join(carpeta, fd)) == real:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode(errors="ignore").strip()
                    otros.append(f"PID {pid}: {cmd[:90]}")
                    break
        except OSError:
            continue   # proceso de otro usuario o que ya terminó
    return otros


class PuenteGazeboReal(Node):
    def __init__(self):
        super().__init__("puente_gazebo_real")

        self.lock = threading.Lock()
        self.objetivo = None   # lo que pide Gazebo (recortado a LIMITES); None hasta que publique
        self.comando = None    # lo que se manda al robot real (con límite de velocidad); arranca
                               # en la posición medida, así reiniciar el puente no mueve el robot
        self.medido = dict(HOME)     # lo que mide el robot real
        self.gazebo_listo = False
        self.t_envio = time.monotonic()
        self.pinza_pendientes = []   # pedidos de /pinza que faltan mandar al ESP32 (en orden)
        self.pinza = None            # último ángulo mandado (para el CSV)

        self.create_subscription(JointState, TOPICO_GAZEBO, self._de_gazebo, 10)
        self.create_subscription(Float64, TOPICO_PINZA, self._de_pinza, 10)
        self.pub_real = self.create_publisher(JointState, TOPICO_REAL, 10)

        otros = procesos_usando(PUERTO)
        if otros:
            self.get_logger().error(f"El puerto {PUERTO} lo tiene abierto otro programa:\n  "
                                    + "\n  ".join(otros)
                                    + "\nCiérrenlo (si es el monitor serial: Ctrl+] en su terminal) "
                                      "y vuelvan a correr el puente.")
            raise SystemExit(1)

        # En Linux el puerto abre con DTR y RTS activos a la vez, y así el ESP32 NO se reinicia.
        # Ponerlos en False lo reinicia: pyserial baja DTR antes que RTS, y ese instante
        # (DTR inactivo, RTS activo) es justo el pulso de reset de la placa.
        self.ser = serial.Serial()
        self.ser.port = PUERTO
        self.ser.baudrate = BAUDIOS
        self.ser.timeout = 0.1
        self.ser.dtr = True
        self.ser.rts = True
        self.ser.open()
        self.ser.reset_input_buffer()   # descartar lo que el ESP32 mandó antes de abrir
        self.reinicio = False           # True si el ESP32 se reinicia: ya no se le manda nada
        self.get_logger().info(f"Puerto {PUERTO} abierto a {BAUDIOS} baudios")
        time.sleep(0.1)
        # El '\n' inicial termina cualquier basura que haya entrado al abrir el puerto.
        # TEL:1 enciende la telemetría y WD:500 detiene los motores si este nodo se cae.
        self.ser.write(b"\nTEL:1\nWD:500\n")

        carpeta = os.path.expanduser("~/ros2_ws")
        if not os.path.isdir(carpeta):
            carpeta = os.getcwd()
        self.ruta_csv = os.path.join(carpeta, datetime.now().strftime("robot_real_%Y%m%d_%H%M%S.csv"))
        self.archivo_csv = open(self.ruta_csv, "w", newline="")
        self.csv = csv.writer(self.archivo_csv)
        self.csv.writerow(["t_s", "t_esp_ms",
                           "gz_q1", "gz_q2", "gz_d3",
                           "cmd_q1", "cmd_q2", "cmd_d3",
                           "real_q1", "real_q2", "real_d3", "pinza_grados"])
        self.t0 = time.monotonic()
        self.get_logger().info(f"Guardando Gazebo vs. robot real en {self.ruta_csv}")
        if not MOVER_Z:
            self.get_logger().warning("MOVER_Z = False: el Z real se queda arriba")

        self.t_tel = time.monotonic()   # último TEL:1 enviado
        self.activo = True
        self.hilo_lector = threading.Thread(target=self._leer_serial, daemon=True)
        self.hilo_lector.start()
        self.create_timer(1.0 / HZ_ENVIO, self._enviar)
        self.get_logger().info(f"Esperando {TOPICO_GAZEBO} de Gazebo...")

    # ---------------- Gazebo -> objetivo ----------------
    def _de_gazebo(self, msg):
        nuevos = {}
        for n in ARTICULACIONES:
            if n in msg.name:
                i = msg.name.index(n)
                if i < len(msg.position) and math.isfinite(msg.position[i]):
                    nuevos[n] = msg.position[i]
        if len(nuevos) < len(ARTICULACIONES):
            self.get_logger().warning(f"{TOPICO_GAZEBO} trae {list(msg.name)}, faltan algunas de "
                                   f"{ARTICULACIONES}: corrijan ARTICULACIONES",
                                   throttle_duration_sec=5.0)
            if not nuevos:
                return

        for n, v in nuevos.items():
            lo, hi = LIMITES[n]
            margen = 0.002 if n == "Joint_3" else 0.01   # Gazebo pasa los topes por un pelo
            if v < lo - margen or v > hi + margen:
                self.get_logger().warning(f"Gazebo pide {n} = {v:.3f}, fuera del rango del robot "
                                          f"real [{lo}, {hi}]: se recorta", throttle_duration_sec=2.0)
            nuevos[n] = min(max(v, lo), hi)
        with self.lock:
            if self.objetivo is None:
                self.objetivo = dict(self.comando or self.medido)
            self.objetivo.update(nuevos)

        if not self.gazebo_listo:
            self.gazebo_listo = True
            pose = ", ".join(f"{n} = {v:.3f}" for n, v in nuevos.items())
            self.get_logger().info(f"Conectado a Gazebo: {pose}")
            lejos = [n for n, v in nuevos.items()
                     if abs(v - HOME[n]) > (0.002 if n == "Joint_3" else 0.05)]
            if lejos:
                self.get_logger().warning(f"Gazebo no está en la pose cero ({', '.join(lejos)}): "
                                       "el robot real va a ir despacio hasta la pose de Gazebo")

    # ---------------- /pinza -> ESP32 ----------------
    def _de_pinza(self, msg):
        if not math.isfinite(msg.data):
            return
        valor = -1.0 if msg.data < 0 else min(msg.data, 180.0)   # negativo = soltar el servo
        with self.lock:
            self.pinza_pendientes.append(valor)

    # ---------------- objetivo -> ESP32 (50 Hz) ----------------
    def _enviar(self):
        if not self.activo:
            return
        ahora = time.monotonic()
        dt = min(ahora - self.t_envio, 0.1)
        self.t_envio = ahora
        with self.lock:
            sin_telemetria = self.comando is None
        if sin_telemetria and not self.reinicio and ahora - self.t_tel > 2.0:
            # Sin posición del ESP32 el robot no se mueve. Volver a pedir la telemetría.
            self.t_tel = ahora
            otros = procesos_usando(PUERTO)
            pista = ("Otro programa tiene abierto el puerto: " + "; ".join(otros)
                     + " (si es el monitor serial, ciérrenlo con Ctrl+])") if otros else \
                    "¿Está conectado y encendido el ESP32?"
            self.get_logger().error(f"No llega la posición del ESP32, el robot real no se va a mover. {pista}")
            try:
                self.ser.write(b"\nTEL:1\n")
            except (serial.SerialException, OSError):
                pass
        with self.lock:
            if self.comando is None or self.reinicio:
                return   # sin lectura de encoders todavía, o el ESP32 perdió el cero
            for n in ARTICULACIONES:
                # Sin Gazebo todavía: sostener la posición actual
                meta = self.objetivo[n] if self.objetivo is not None else self.comando[n]
                if n == "Joint_3" and not MOVER_Z:
                    meta = HOME["Joint_3"]
                paso = VEL_MAX[n] * dt
                self.comando[n] += max(-paso, min(paso, meta - self.comando[n]))
            cmd = dict(self.comando)
            pinzas, self.pinza_pendientes = self.pinza_pendientes, []

        por_motor = [0, 0, 0]
        for n in ARTICULACIONES:
            por_motor[MOTOR_DE[n] - 1] = a_cuentas(n, cmd[n])
        linea = f"SET:{por_motor[0]},{por_motor[1]},{por_motor[2]}\n"
        for v in pinzas:
            linea += "SERVO:OFF\n" if v < 0 else f"SERVO:{v:.0f}\n"
        try:
            self.ser.write(linea.encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"No se pudo escribir al ESP32: {e}", throttle_duration_sec=2.0)
            return
        for v in pinzas:
            if v < 0:
                self.get_logger().info("Pinza suelta (servo sin pulsos)")
            else:
                self.pinza = v
                self.get_logger().info(f"Pinza -> {v:.0f} grados")

    # ---------------- ESP32 -> /real/joint_states + CSV ----------------
    def _leer_serial(self):
        while self.activo:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except (serial.SerialException, OSError, TypeError) as e:
                if not self.activo:
                    return   # cierre normal del puerto
                # No rendirse: si otro programa soltó el puerto o fue un error suelto, sigue
                otros = procesos_usando(PUERTO)
                pista = (" Otro programa tiene abierto el puerto: " + "; ".join(otros)
                         + " (si es el monitor serial, ciérrenlo con Ctrl+]).") if otros else ""
                self.get_logger().error(f"Error leyendo el ESP32: {e}.{pista}", throttle_duration_sec=3.0)
                time.sleep(0.5)
                continue
            if not linea.startswith("POS:"):
                if "SCARA R4 listo" in linea and not self.reinicio:
                    self.reinicio = True
                    self.get_logger().error(
                        "El ESP32 se acaba de reiniciar: sus cuentas volvieron a 0 y el robot NO se "
                        "va a mover. Si el robot está físicamente en la pose cero, cierren con Ctrl+C "
                        "y vuelvan a correr el puente; si no, pónganlo en cero con el monitor serial.")
                elif linea.startswith("ERR:FALLA"):
                    self.get_logger().error(
                        f"ESP32: {linea}\nEl ESP32 apagó los motores por seguridad y el robot real ya no "
                        "sigue a Gazebo. Revisen el motor y el encoder que dice el mensaje; después "
                        "Ctrl+C aquí (manda STOP) y vuelvan a correr el puente.")
                elif linea.startswith("ERR:en FALLA"):
                    pass   # ya se avisó arriba; el ESP32 lo repite cada segundo
                elif linea:
                    self.get_logger().info(f"ESP32: {linea}")
                continue
            try:
                campos = [int(x) for x in linea[4:].split(",")]
                t_ms, cuentas = campos[0], campos[1:4]
            except (ValueError, IndexError):
                continue
            if len(cuentas) < 3:
                continue

            medido = {n: de_cuentas(n, cuentas[MOTOR_DE[n] - 1]) for n in ARTICULACIONES}
            with self.lock:
                self.medido = medido
                if self.comando is None:
                    self.comando = dict(medido)
                    pose = ", ".join(f"{n} = {v:.3f}" for n, v in medido.items())
                    self.get_logger().info(f"Robot real en: {pose}")
                gz = dict(self.objetivo) if self.objetivo is not None else None
                cmd = dict(self.comando)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(ARTICULACIONES)
            msg.position = [medido[n] for n in ARTICULACIONES]
            try:
                self.pub_real.publish(msg)
            except Exception:
                return   # ROS ya se cerró (Ctrl+C): termina la lectura sin error

            if gz is None:
                continue   # el CSV empieza cuando Gazebo esté conectado
            self.csv.writerow([f"{time.monotonic() - self.t0:.4f}", t_ms,
                               *(f"{gz[n]:.5f}" for n in ARTICULACIONES),
                               *(f"{cmd[n]:.5f}" for n in ARTICULACIONES),
                               *(f"{medido[n]:.5f}" for n in ARTICULACIONES),
                               "" if self.pinza is None else f"{self.pinza:.0f}"])

    def cerrar(self):
        self.activo = False
        self.hilo_lector.join(timeout=0.5)   # esperar a que termine la lectura en curso
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
            self.ser.close()
        finally:
            self.archivo_csv.close()
            print(f"CSV guardado: {self.ruta_csv}")   # print: después de Ctrl+C el logger de ROS ya cerró


def main(args=None):
    rclpy.init(args=args)
    nodo = PuenteGazeboReal()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
