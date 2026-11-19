
#!/usr/bin/env python3

import csv
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
]

BASE = os.path.expanduser(
    "~/robot2_ws"
)

CSV_MATLAB = os.path.join(
    BASE,
    "trayectorias",
    "trayectoria_A_B_v2.csv"
)

CSV_GAZEBO = os.path.join(
    BASE,
    "resultados",
    "posiciones_gazebo.csv"
)

CARPETA_RESULTADOS = os.path.join(
    BASE,
    "resultados"
)


def leer_csv(ruta):
    tiempos = []
    posiciones = []

    with open(
        ruta,
        newline="",
        encoding="utf-8-sig"
    ) as archivo:

        lector = csv.DictReader(archivo)

        for fila in lector:
            tiempos.append(
                float(fila["time_from_start"])
            )

            posiciones.append([
                float(fila[j])
                for j in JOINTS
            ])

    t = np.array(
        tiempos,
        dtype=float
    )

    q = np.array(
        posiciones,
        dtype=float
    )

    if len(t) < 2:
        raise ValueError(
            f"Datos insuficientes en {ruta}"
        )

    if not (
        np.all(np.isfinite(t))
        and np.all(np.isfinite(q))
    ):
        raise ValueError(
            f"Datos no finitos en {ruta}"
        )

    if np.any(np.diff(t) <= 0):
        raise ValueError(
            f"Tiempos no crecientes en {ruta}"
        )

    return t, q


def main():
    os.makedirs(
        CARPETA_RESULTADOS,
        exist_ok=True
    )

    t_matlab, q_matlab = leer_csv(
        CSV_MATLAB
    )

    t_gazebo, q_gazebo = leer_csv(
        CSV_GAZEBO
    )

    # Comparar solo el intervalo comun.
    t_inicio = max(
        t_matlab[0],
        t_gazebo[0]
    )

    t_fin = min(
        t_matlab[-1],
        t_gazebo[-1]
    )

    mascara = (
        (t_gazebo >= t_inicio)
        & (t_gazebo <= t_fin)
    )

    t_real = t_gazebo[mascara]
    q_real = q_gazebo[mascara]

    if len(t_real) < 10:
        raise ValueError(
            "Hay menos de 10 muestras en "
            "el intervalo comun."
        )

    # Interpolar posiciones deseadas
    # a los tiempos medidos en Gazebo.
    q_deseada = np.column_stack([
        np.interp(
            t_real,
            t_matlab,
            q_matlab[:, i]
        )
        for i in range(4)
    ])

    # Convertir de radianes a grados.
    deseada_deg = np.rad2deg(
        q_deseada
    )

    real_deg = np.rad2deg(
        q_real
    )

    error_deg = (
        real_deg - deseada_deg
    )

    rmse = np.sqrt(
        np.mean(
            error_deg ** 2,
            axis=0
        )
    )

    mae = np.mean(
        np.abs(error_deg),
        axis=0
    )

    error_maximo = np.max(
        np.abs(error_deg),
        axis=0
    )

    print("\n" + "=" * 60)
    print("RESULTADOS MATLAB VS GAZEBO")
    print("=" * 60)

    print(
        "Muestras comparadas:",
        len(t_real)
    )

    print(
        "Intervalo comparado:",
        round(t_real[0], 3),
        "a",
        round(t_real[-1], 3),
        "s"
    )

    print(
        "\nArticulacion | RMSE (deg) | MAE (deg) | Max (deg)"
    )

    print("-" * 60)

    for i, nombre in enumerate(JOINTS):
        print(
            f"{nombre:12s} | "
            f"{rmse[i]:10.4f} | "
            f"{mae[i]:9.4f} | "
            f"{error_maximo[i]:9.4f}"
        )

    # ====================================
    # GRAFICAS DE POSICION
    # ====================================

    for i, nombre in enumerate(JOINTS):
        plt.figure(
            figsize=(10, 5)
        )

        plt.plot(
            t_matlab,
            np.rad2deg(q_matlab[:, i]),
            label="MATLAB - Deseada",
            linewidth=2
        )

        plt.plot(
            t_real,
            real_deg[:, i],
            label="Gazebo - Real",
            linestyle="--",
            linewidth=1.5
        )

        plt.xlabel(
            "Tiempo [s]"
        )

        plt.ylabel(
            "Posicion angular [grados]"
        )

        plt.title(
            f"Validacion de posicion - {nombre}"
        )

        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        ruta = os.path.join(
            CARPETA_RESULTADOS,
            f"comparacion_{nombre}.png"
        )

        plt.savefig(
            ruta,
            dpi=160
        )

        plt.close()

    # ====================================
    # GRAFICA DE ERRORES
    # ====================================

    plt.figure(
        figsize=(11, 6)
    )

    for i, nombre in enumerate(JOINTS):
        plt.plot(
            t_real,
            error_deg[:, i],
            label=nombre
        )

    plt.axhline(
        0,
        linestyle=":",
        linewidth=1
    )

    plt.xlabel(
        "Tiempo [s]"
    )

    plt.ylabel(
        "Error angular [grados]"
    )

    plt.title(
        "Error de seguimiento MATLAB vs Gazebo"
    )

    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        os.path.join(
            CARPETA_RESULTADOS,
            "errores_articulaciones.png"
        ),
        dpi=160
    )

    plt.close()

    # ====================================
    # CSV CON COMPARACION COMPLETA
    # ====================================

    ruta_comparacion = os.path.join(
        CARPETA_RESULTADOS,
        "comparacion_completa.csv"
    )

    encabezados = [
        "time_from_start"
    ]

    for nombre in JOINTS:
        encabezados.extend([
            f"{nombre}_deseada_deg",
            f"{nombre}_real_deg",
            f"{nombre}_error_deg",
        ])

    with open(
        ruta_comparacion,
        "w",
        newline="",
        encoding="utf-8"
    ) as archivo:

        escritor = csv.writer(
            archivo
        )

        escritor.writerow(
            encabezados
        )

        for k, t in enumerate(t_real):
            fila = [float(t)]

            for i in range(4):
                fila.extend([
                    float(deseada_deg[k, i]),
                    float(real_deg[k, i]),
                    float(error_deg[k, i]),
                ])

            escritor.writerow(
                fila
            )

    # ====================================
    # CSV DE METRICAS
    # ====================================

    ruta_metricas = os.path.join(
        CARPETA_RESULTADOS,
        "metricas_error.csv"
    )

    with open(
        ruta_metricas,
        "w",
        newline="",
        encoding="utf-8"
    ) as archivo:

        escritor = csv.writer(
            archivo
        )

        escritor.writerow([
            "joint",
            "rmse_deg",
            "mae_deg",
            "max_abs_error_deg",
        ])

        for i, nombre in enumerate(JOINTS):
            escritor.writerow([
                nombre,
                float(rmse[i]),
                float(mae[i]),
                float(error_maximo[i]),
            ])

    print("\nArchivos generados en:")
    print(CARPETA_RESULTADOS)

    print("\nGraficas:")
    for nombre in JOINTS:
        print(
            f"  comparacion_{nombre}.png"
        )

    print(
        "  errores_articulaciones.png"
    )

    print("\nTablas:")
    print(
        "  comparacion_completa.csv"
    )
    print(
        "  metricas_error.csv"
    )


if __name__ == "__main__":
    main()
