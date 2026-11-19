
#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


# ============================================
# CONFIGURACION GENERAL
# ============================================

JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
]

CSV_DEFAULT = os.path.expanduser(
    "~/robot2_ws/trayectorias/trayectoria_A_B_v2.csv"
)

ACTION_NAME = "/arm_controller/follow_joint_trajectory"

# HOME DEFINITIVO
HOME = [
    0.0,                    # joint_1 = 0 grados
    0.0,                    # joint_2 = 0 grados
    math.radians(-180.0),   # joint_3 = -180 grados
    0.0,                    # joint_4 = 0 grados
]

# Limites provisionales del modelo MATLAB
LIMITS = [
    (-math.pi, math.pi),
    (-math.pi / 2, math.pi / 2),
    (-math.pi, 0.0),
    (math.radians(-150.0), math.pi / 2),
]

# Duraciones de movimientos
TIEMPO_IR_A = 8.0
TIEMPO_GIRO = 5.0
TIEMPO_DESHACER = 5.0
TIEMPO_HOME = 10.0

# Giro relativo de joint_4 respecto al punto B
GIRO_JOINT4 = math.radians(-135.0)

# Tolerancia para verificar posicion final
TOLERANCIA_POS = math.radians(2.0)

# Primer punto de las trayectorias cortas
TIEMPO_PRIMER_PUNTO = 0.1


# ============================================
# FUNCIONES AUXILIARES
# ============================================

def grados(q):
    return [
        round(math.degrees(x), 2)
        for x in q
    ]


def distancia_maxima(q1, q2):
    return max(
        abs(a - b)
        for a, b in zip(q1, q2)
    )


def validar_limites(q, descripcion):
    if len(q) != 4:
        raise ValueError(
            f"{descripcion}: se esperaban 4 articulaciones."
        )

    for i, valor in enumerate(q):
        minimo, maximo = LIMITS[i]

        if not math.isfinite(valor):
            raise ValueError(
                f"{descripcion}: valor no finito en {JOINTS[i]}"
            )

        if valor < minimo - 1e-6 or valor > maximo + 1e-6:
            raise ValueError(
                f"{descripcion}: {JOINTS[i]} fuera de limites. "
                f"Valor: {math.degrees(valor):.3f} grados"
            )


MODO_AUTO = False


def confirmar(texto):
    print("\n" + "=" * 55)
    print(texto)
    print("=" * 55)

    if MODO_AUTO:
        print("MODO AUTOMATICO: continuando sin confirmacion.")
        return True

    respuesta = input(
        "Escribe SI para continuar: "
    ).strip().upper()

    if respuesta != "SI":
        print("Secuencia detenida por el usuario.")
        return False

    return True


# ============================================
# LEER Y VALIDAR CSV DE MATLAB
# ============================================

def leer_csv(ruta):
    if not os.path.isfile(ruta):
        raise FileNotFoundError(
            f"No existe el archivo CSV: {ruta}"
        )

    columnas = (
        ["time_from_start"]
        + JOINTS
        + [f"velocity_{i}" for i in range(1, 5)]
        + [f"acceleration_{i}" for i in range(1, 5)]
    )

    puntos = []

    with open(
        ruta,
        newline="",
        encoding="utf-8-sig"
    ) as archivo:

        lector = csv.DictReader(archivo)

        if lector.fieldnames is None:
            raise ValueError(
                "El CSV no tiene encabezados."
            )

        faltantes = [
            c for c in columnas
            if c not in lector.fieldnames
        ]

        if faltantes:
            raise ValueError(
                f"Faltan columnas: {faltantes}"
            )

        for numero, fila in enumerate(lector, start=2):
            try:
                t = float(
                    fila["time_from_start"]
                )

                q = [
                    float(fila[j])
                    for j in JOINTS
                ]

                # Leemos velocidades y aceleraciones
                # para validar las 13 columnas,
                # pero no las enviamos al controlador.
                v = [
                    float(fila[f"velocity_{i}"])
                    for i in range(1, 5)
                ]

                a = [
                    float(fila[f"acceleration_{i}"])
                    for i in range(1, 5)
                ]

            except (ValueError, TypeError) as error:
                raise ValueError(
                    f"Error leyendo fila {numero}"
                ) from error

            valores = [t] + q + v + a

            if not all(
                math.isfinite(x)
                for x in valores
            ):
                raise ValueError(
                    f"Valor no finito en fila {numero}"
                )

            validar_limites(
                q,
                f"Fila {numero}"
            )

            if puntos and t <= puntos[-1][0]:
                raise ValueError(
                    f"Tiempo no creciente en fila {numero}"
                )

            puntos.append(
                (t, q)
            )

    if len(puntos) < 2:
        raise ValueError(
            "La trayectoria tiene menos de dos puntos."
        )

    if abs(puntos[0][0]) > 1e-8:
        raise ValueError(
            "El CSV debe comenzar en tiempo cero."
        )

    return puntos


# ============================================
# NODO ROS 2
# ============================================

class Ejecutor(Node):

    def __init__(self):
        super().__init__(
            "ejecutor_csv_robot2"
        )

        self.cliente = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME
        )

        self.estado_actual = None

        self.suscripcion = self.create_subscription(
            JointState,
            "/joint_states",
            self.recibir_estado,
            10
        )

    # ----------------------------------------
    # RECIBIR ESTADO
    # ----------------------------------------

    def recibir_estado(self, mensaje):
        if not all(
            j in mensaje.name
            for j in JOINTS
        ):
            return

        indices = [
            mensaje.name.index(j)
            for j in JOINTS
        ]

        if any(
            i >= len(mensaje.position)
            for i in indices
        ):
            return

        posiciones = [
            float(mensaje.position[i])
            for i in indices
        ]

        if all(
            math.isfinite(x)
            for x in posiciones
        ):
            self.estado_actual = posiciones

    # ----------------------------------------
    # LEER POSICION ACTUAL
    # ----------------------------------------

    def leer_estado_fresco(self):
        self.estado_actual = None

        for _ in range(100):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.estado_actual is not None:
                return list(
                    self.estado_actual
                )

        raise RuntimeError(
            "No se reciben posiciones en /joint_states."
        )

    # ----------------------------------------
    # ESPERAR CONTROLADOR
    # ----------------------------------------

    def esperar_controlador(self):
        disponible = self.cliente.wait_for_server(
            timeout_sec=8.0
        )

        if not disponible:
            raise RuntimeError(
                "No responde el controlador: "
                + ACTION_NAME
            )

    # ----------------------------------------
    # VERIFICAR POSICION ALCANZADA
    # ----------------------------------------

    def verificar_posicion(self, objetivo, nombre):
        import time

        print(f"\\nVerificacion: {nombre}")
        print("Esperando estabilizacion (maximo 5 segundos)...")

        limite = time.monotonic() + 5.0

        while True:
            actual = self.leer_estado_fresco()
            error = distancia_maxima(actual, objetivo)

            if error <= TOLERANCIA_POS:
                break

            if time.monotonic() >= limite:
                break

        print("Objetivo [grados]:", grados(objetivo))
        print("Actual   [grados]:", grados(actual))
        print(
            "Error maximo [grados]:",
            round(math.degrees(error), 4)
        )

        if error > TOLERANCIA_POS:
            raise RuntimeError(
                f"El robot no alcanzo {nombre} "
                f"dentro de la tolerancia."
            )

        return actual

    # ----------------------------------------
    # ENVIAR TRAYECTORIA
    # ----------------------------------------

    def enviar(
        self,
        puntos,
        descripcion
    ):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINTS

        # Enviar SOLO posiciones y tiempos.
        # Esta fue la configuracion que funciono
        # con el controlador de Gazebo.

        tiempo_anterior = -1.0

        for t, q in puntos:
            if t <= 0:
                raise ValueError(
                    "Los tiempos deben ser positivos."
                )

            if t <= tiempo_anterior:
                raise ValueError(
                    "Los tiempos enviados deben crecer."
                )

            tiempo_anterior = t

            validar_limites(
                q,
                descripcion
            )

            punto = JointTrajectoryPoint()

            punto.positions = list(q)

            punto.time_from_start = Duration(
                seconds=float(t)
            ).to_msg()

            goal.trajectory.points.append(
                punto
            )

        self.get_logger().info(
            f"Enviando {descripcion}: "
            f"{len(puntos)} puntos"
        )

        futuro = self.cliente.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            futuro
        )

        handle = futuro.result()

        if handle is None or not handle.accepted:
            raise RuntimeError(
                f"El controlador rechazo: {descripcion}"
            )

        self.get_logger().info(
            f"{descripcion}: objetivo aceptado."
        )

        resultado_futuro = handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            resultado_futuro
        )

        respuesta = resultado_futuro.result()

        if respuesta is None:
            raise RuntimeError(
                f"No se recibio resultado para {descripcion}"
            )

        resultado = respuesta.result

        if resultado.error_code != (
            FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f"Fallo en {descripcion}. "
                f"Codigo: {resultado.error_code}. "
                f"Detalle: {resultado.error_string}"
            )

        self.get_logger().info(
            f"{descripcion}: finalizado correctamente."
        )

    # ----------------------------------------
    # MOVIMIENTO ENTRE DOS POSTURAS
    # ----------------------------------------

    def mover_articulaciones(
        self,
        objetivo,
        duracion,
        descripcion
    ):
        actual = self.leer_estado_fresco()

        validar_limites(
            objetivo,
            descripcion
        )

        print("\nMovimiento:", descripcion)

        print(
            "Desde [grados]:",
            grados(actual)
        )

        print(
            "Hasta [grados]:",
            grados(objetivo)
        )

        print(
            "Duracion:",
            duracion,
            "segundos"
        )

        # Si ya esta en el objetivo,
        # no enviamos un movimiento adicional.
        if distancia_maxima(
            actual,
            objetivo
        ) < math.radians(0.1):

            print(
                "El robot ya esta en esta posicion."
            )

            return self.verificar_posicion(
                objetivo,
                descripcion
            )

        puntos = [
            (
                TIEMPO_PRIMER_PUNTO,
                actual
            ),
            (
                duracion,
                objetivo
            )
        ]

        self.enviar(
            puntos,
            descripcion
        )

        return self.verificar_posicion(
            objetivo,
            descripcion
        )


# ============================================
# PROGRAMA PRINCIPAL
# ============================================

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--csv",
        default=CSV_DEFAULT
    )

    parser.add_argument(
        "--check",
        action="store_true"
    )

    parser.add_argument(
        "--run",
        action="store_true"
    )

    parser.add_argument(
        "--auto",
        action="store_true",
        help="Ejecutar sin confirmaciones, solo en Gazebo"
    )

    args = parser.parse_args()

    global MODO_AUTO
    MODO_AUTO = args.auto

    if args.auto and not args.run:
        parser.error("--auto requiere --run")


    if not args.check and not args.run:
        parser.error(
            "Debes indicar --check o --run"
        )

    # Leer trayectoria de MATLAB
    puntos = leer_csv(
        args.csv
    )

    qA = list(
        puntos[0][1]
    )

    qB = list(
        puntos[-1][1]
    )

    # Punto de vertido:
    # joint_4 gira -135 grados respecto a B.
    qVertido = list(qB)

    qVertido[3] += GIRO_JOINT4

    validar_limites(
        qVertido,
        "Punto de vertido"
    )

    validar_limites(
        HOME,
        "HOME"
    )

    print("\n" + "=" * 55)
    print("ROBOT2 - SECUENCIA COMPLETA")
    print("=" * 55)

    print(
        "Archivo CSV:",
        args.csv
    )

    print(
        "Muestras MATLAB:",
        len(puntos)
    )

    print(
        "Duracion A-B:",
        puntos[-1][0],
        "segundos"
    )

    print("\nPunto A [grados]:")
    print(grados(qA))

    print("\nPunto B [grados]:")
    print(grados(qB))

    print("\nPunto de vertido [grados]:")
    print(grados(qVertido))

    print("\nHOME definitivo [grados]:")
    print(grados(HOME))

    print("\nSECUENCIA:")
    print("1. Posicionamiento en A")
    print("2. Trayectoria MATLAB A -> B")
    print("3. Giro joint_4 de -135 grados")
    print("4. Deshacer giro de joint_4")
    print("5. Regreso al HOME definitivo")

    rclpy.init()

    nodo = Ejecutor()

    try:
        nodo.esperar_controlador()

        actual = nodo.leer_estado_fresco()

        print("\nArticulaciones actuales [grados]:")
        print(grados(actual))

        # ====================================
        # MODO VERIFICACION
        # ====================================

        if args.check:
            print(
                "\nVERIFICACION CORRECTA."
            )

            print(
                "No se ha movido el robot."
            )

            return

        # ====================================
        # MODO EJECUCION
        # ====================================

        print("\nADVERTENCIA:")
        print(
            "Los recorridos nuevos no estan "
            "validados completamente contra colisiones."
        )

        print(
            "Utilizar solamente en Gazebo."
        )

        print(
            "Cada fase requiere confirmacion."
        )

        # ====================================
        # FASE 1: IR AL PUNTO A
        # ====================================

        if distancia_maxima(
            actual,
            qA
        ) >= math.radians(0.1):

            if not confirmar(
                "FASE 1: mover el robot hasta A "
                "en 8 segundos. "
                "Revisa visualmente el recorrido."
            ):
                return

            nodo.mover_articulaciones(
                qA,
                TIEMPO_IR_A,
                "Posicionamiento en A"
            )

        else:
            print(
                "\nFASE 1: el robot ya esta en A."
            )

            nodo.verificar_posicion(
                qA,
                "Punto A"
            )

        # ====================================
        # FASE 2: A -> B
        # ====================================

        if not confirmar(
            "FASE 2: ejecutar la trayectoria "
            "MATLAB A -> B durante 17 segundos."
        ):
            return

        # Omitimos el punto t=0 porque el
        # controlador ya acepto asi el CSV.
        trayectoria = puntos[1:]

        nodo.enviar(
            trayectoria,
            "Trayectoria MATLAB A-B V2"
        )

        nodo.verificar_posicion(
            qB,
            "Punto B"
        )

        # ====================================
        # FASE 3: GIRO DE JOINT_4
        # ====================================

        if not confirmar(
            "FASE 3: girar joint_4 -135 grados "
            "respecto a B durante 5 segundos. "
            "Comprueba el espacio de giro."
        ):
            return

        nodo.mover_articulaciones(
            qVertido,
            TIEMPO_GIRO,
            "Giro de joint_4 en B"
        )

        # ====================================
        # FASE 4: DESHACER GIRO
        # ====================================

        if not confirmar(
            "FASE 4: regresar joint_4 "
            "a su posicion original en B "
            "durante 5 segundos."
        ):
            return

        nodo.mover_articulaciones(
            qB,
            TIEMPO_DESHACER,
            "Deshacer giro de joint_4"
        )

        # ====================================
        # FASE 5: REGRESO A HOME
        # ====================================

        if not confirmar(
            "FASE 5: regresar desde B hasta "
            "HOME [0, 0, -180, 0] grados "
            "durante 10 segundos. "
            "Este recorrido directo aun debe "
            "verificarse contra colisiones."
        ):
            return

        nodo.mover_articulaciones(
            HOME,
            TIEMPO_HOME,
            "Regreso al HOME definitivo"
        )

        print("\n" + "=" * 55)
        print("SECUENCIA COMPLETA FINALIZADA")
        print("=" * 55)

        print("Trayectoria A -> B: completada")
        print("Giro de joint_4: completado")
        print("Deshacer giro: completado")
        print("Regreso a HOME: completado")

        print("\nHOME final [grados]:")
        print(grados(HOME))

    finally:
        nodo.destroy_node()
        rclpy.shutdown()


# ============================================
# INICIO DEL PROGRAMA
# ============================================

if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:
        print(
            "\nPrograma interrumpido."
        )
        sys.exit(130)

    except Exception as error:
        print(
            f"\nERROR: {error}",
            file=sys.stderr
        )
        sys.exit(1)
