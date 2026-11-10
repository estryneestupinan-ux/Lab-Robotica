#!/usr/bin/env python3

import csv
import os
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

RUTA = os.path.expanduser(
    "~/robot2_ws/trayectorias/trayectoria_A_B_v2.csv"
)

rclpy.init()
nodo = Node("prueba_ab_posiciones")

cliente = ActionClient(
    nodo,
    FollowJointTrajectory,
    "/arm_controller/follow_joint_trajectory"
)

if not cliente.wait_for_server(timeout_sec=8.0):
    print("ERROR: controlador no disponible")
else:
    objetivo = FollowJointTrajectory.Goal()
    objetivo.trajectory.joint_names = [
        "joint_1", "joint_2", "joint_3", "joint_4"
    ]

    with open(RUTA, newline="") as archivo:
        filas = list(csv.DictReader(archivo))

    # Omitimos t=0 porque el robot ya está en A.
    for fila in filas[1:]:
        punto = JointTrajectoryPoint()

        punto.positions = [
            float(fila[f"joint_{i}"])
            for i in range(1, 5)
        ]

        punto.time_from_start = Duration(
            seconds=float(fila["time_from_start"])
        ).to_msg()

        objetivo.trajectory.points.append(punto)

    print(
        f"Trayectoria preparada: "
        f"{len(objetivo.trajectory.points)} puntos"
    )

    respuesta = input(
        "Escribe SI para ejecutar A-B: "
    ).strip()

    if respuesta == "SI":
        futuro = cliente.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(nodo, futuro)

        handle = futuro.result()

        if handle is None or not handle.accepted:
            print("ERROR: el controlador rechazó el objetivo")
        else:
            print("Objetivo aceptado. Ejecutando trayectoria...")

            resultado = handle.get_result_async()
            rclpy.spin_until_future_complete(nodo, resultado)

            print(
                "Código de resultado:",
                resultado.result().result.error_code
            )
    else:
        print("Prueba cancelada")

nodo.destroy_node()
rclpy.shutdown()
