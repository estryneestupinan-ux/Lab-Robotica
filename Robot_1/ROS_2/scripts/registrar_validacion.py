
#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
]

CSV_MATLAB = os.path.expanduser(
    "~/robot2_ws/trayectorias/trayectoria_A_B_v2.csv"
)

CSV_REAL = os.path.expanduser(
    "~/robot2_ws/resultados/posiciones_gazebo.csv"
)

ACCION = "/arm_controller/follow_joint_trajectory"

TOLERANCIA_A = math.radians(2.0)
ANTICIPACION = 3.0
MARGEN_FINAL = 1.0


def leer_trayectoria(ruta):
    puntos = []

    with open(
        ruta,
        newline="",
        encoding="utf-8-sig"
    ) as archivo:

        lector = csv.DictReader(archivo)

        columnas = ["time_from_start"] + JOINTS

        if lector.fieldnames is None:
            raise ValueError("El CSV no tiene encabezados.")

        for columna in columnas:
            if columna not in lector.fieldnames:
                raise ValueError(
                    f"Falta la columna {columna}"
                )

        for fila in lector:
            t = float(fila["time_from_start"])
            q = [float(fila[j]) for j in JOINTS]

            if not all(math.isfinite(x) for x in [t] + q):
                raise ValueError(
                    "El CSV contiene valores no finitos."
                )

            if puntos and t <= puntos[-1][0]:
                raise ValueError(
                    "Los tiempos del CSV no son crecientes."
                )

            puntos.append((t, q))

    if len(puntos) < 2:
        raise ValueError("La trayectoria esta vacia.")

    if abs(puntos[0][0]) > 1e-8:
        raise ValueError(
            "La trayectoria debe comenzar en t=0."
        )

    return puntos


class Registrador(Node):

    def __init__(self):
        super().__init__(
            "registrador_validacion_robot2",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True
                )
            ]
        )

        self.cliente = ActionClient(
            self,
            FollowJointTrajectory,
            ACCION
        )

        self.ultima_posicion = None
        self.mediciones = []
        self.inicio_ns = None
        self.fin_ns = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.recibir_estado,
            100
        )

    def recibir_estado(self, mensaje):
        if not all(j in mensaje.name for j in JOINTS):
            return

        indices = [
            mensaje.name.index(j)
            for j in JOINTS
        ]

        if any(i >= len(mensaje.position) for i in indices):
            return

        q = [
            float(mensaje.position[i])
            for i in indices
        ]

        if not all(math.isfinite(x) for x in q):
            return

        self.ultima_posicion = q

        # Usamos el timestamp del propio JointState.
        marca_ns = (
            int(mensaje.header.stamp.sec) * 1_000_000_000
            + int(mensaje.header.stamp.nanosec)
        )

        if self.inicio_ns is None:
            return

        if self.inicio_ns <= marca_ns <= self.fin_ns:
            t = (marca_ns - self.inicio_ns) / 1e9

            self.mediciones.append(
                [t] + q
            )

    def esperar_estado(self):
        self.ultima_posicion = None

        limite = time.monotonic() + 12.0

        while time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.ultima_posicion is not None:
                return list(self.ultima_posicion)

        raise RuntimeError(
            "No llegan datos de /joint_states."
        )

    def esperar_reloj(self):
        limite = time.monotonic() + 12.0

        while time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.1)

            ahora = self.get_clock().now().nanoseconds

            if ahora > 0:
                return ahora

        raise RuntimeError(
            "No avanza el reloj de simulacion /clock."
        )

    def validar_punto_a(self, q_a):
        actual = self.esperar_estado()

        errores = [
            abs(x - y)
            for x, y in zip(actual, q_a)
        ]

        print("\nPosicion actual [grados]:")
        print([
            round(math.degrees(x), 2)
            for x in actual
        ])

        print("\nPunto A de MATLAB [grados]:")
        print([
            round(math.degrees(x), 2)
            for x in q_a
        ])

        if max(errores) > TOLERANCIA_A:
            raise RuntimeError(
                "El robot NO esta en A. "
                "Primero debes posicionarlo en A."
            )

        print("\nEl robot esta en A. Correcto.")

    def ejecutar_y_registrar(self, puntos):
        if not self.cliente.wait_for_server(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                "No responde arm_controller."
            )

        duracion = puntos[-1][0]

        ahora_ns = self.esperar_reloj()

        self.inicio_ns = (
            ahora_ns
            + int(ANTICIPACION * 1e9)
        )

        self.fin_ns = (
            self.inicio_ns
            + int((duracion + MARGEN_FINAL) * 1e9)
        )

        objetivo = FollowJointTrajectory.Goal()
        objetivo.trajectory.joint_names = JOINTS

        # El controlador ejecutara la trayectoria
        # en este instante futuro del reloj ROS.
        inicio_ros = Time(
            nanoseconds=self.inicio_ns
        )

        objetivo.trajectory.header.stamp = (
            inicio_ros.to_msg()
        )

        # Omitimos t=0, igual que en el programa
        # que ya funciona con tu controlador.
        for t, q in puntos[1:]:
            punto = JointTrajectoryPoint()
            punto.positions = q
            punto.time_from_start = Duration(
                seconds=float(t)
            ).to_msg()

            objetivo.trajectory.points.append(
                punto
            )

        print("\nEnviando trayectoria A -> B...")
        print(
            f"Duracion programada: {duracion:.2f} s"
        )
        print(
            "El registro usa tiempo de simulacion."
        )

        futuro = self.cliente.send_goal_async(
            objetivo
        )

        rclpy.spin_until_future_complete(
            self,
            futuro,
            timeout_sec=10.0
        )

        if not futuro.done():
            raise RuntimeError(
                "No hubo respuesta del controlador."
            )

        handle = futuro.result()

        if handle is None or not handle.accepted:
            raise RuntimeError(
                "El controlador rechazo la trayectoria."
            )

        print("Trayectoria aceptada.")

        resultado_futuro = handle.get_result_async()

        limite_real = (
            time.monotonic()
            + max(40.0, duracion * 4.0)
        )

        # Spin continuo: permite recibir JointState
        # mientras se ejecuta la trayectoria.
        while (
            not resultado_futuro.done()
            and time.monotonic() < limite_real
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )

        if not resultado_futuro.done():
            raise RuntimeError(
                "Tiempo de espera agotado. "
                "Revisa si Gazebo esta pausado."
            )

        respuesta = resultado_futuro.result()

        if respuesta is None:
            raise RuntimeError(
                "No se recibio resultado."
            )

        resultado = respuesta.result

        if resultado.error_code != (
            FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f"Error del controlador: "
                f"{resultado.error_code} "
                f"{resultado.error_string}"
            )

        # Capturar brevemente las ultimas muestras.
        fin_espera = time.monotonic() + 1.0

        while time.monotonic() < fin_espera:
            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )

        print("\nTrayectoria completada.")

    def guardar_csv(self, ruta):
        if not self.mediciones:
            raise RuntimeError(
                "No se registraron mediciones."
            )

        # Ordenar y eliminar timestamps duplicados.
        ordenadas = sorted(
            self.mediciones,
            key=lambda fila: fila[0]
        )

        limpias = []
        ultimo_t = None

        for fila in ordenadas:
            t = fila[0]

            if ultimo_t is not None and abs(
                t - ultimo_t
            ) < 1e-9:
                continue

            limpias.append(fila)
            ultimo_t = t

        os.makedirs(
            os.path.dirname(ruta),
            exist_ok=True
        )

        with open(
            ruta,
            "w",
            newline="",
            encoding="utf-8"
        ) as archivo:

            escritor = csv.writer(archivo)

            escritor.writerow(
                ["time_from_start"] + JOINTS
            )

            escritor.writerows(limpias)

        print("\nCSV registrado:")
        print(ruta)
        print(
            "Numero de muestras:",
            len(limpias)
        )

        print(
            "Intervalo registrado [s]:",
            round(limpias[0][0], 3),
            "a",
            round(limpias[-1][0], 3)
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--csv",
        default=CSV_MATLAB
    )

    parser.add_argument(
        "--salida",
        default=CSV_REAL
    )

    args = parser.parse_args()

    puntos = leer_trayectoria(args.csv)

    print("\nVALIDACION MATLAB VS GAZEBO")
    print(
        "Puntos de MATLAB:",
        len(puntos)
    )

    print(
        "Duracion:",
        puntos[-1][0],
        "segundos"
    )

    rclpy.init()
    nodo = Registrador()

    try:
        nodo.validar_punto_a(
            puntos[0][1]
        )

        print(
            "\nATENCION: esta prueba movera "
            "el robot de A hasta B."
        )

        respuesta = input(
            "Escribe SI para ejecutar y registrar: "
        ).strip().upper()

        if respuesta != "SI":
            print("Prueba cancelada.")
            return

        try:
            nodo.ejecutar_y_registrar(puntos)
        finally:
            if nodo.mediciones:
                nodo.guardar_csv(args.salida)

    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")

    except Exception as error:
        print(f"\nERROR: {error}")
        raise SystemExit(1)
