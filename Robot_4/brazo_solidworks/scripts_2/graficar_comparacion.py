#!/usr/bin/env python3
"""
Gráfica Gazebo vs. robot real a partir del CSV que guarda puente_serial.py.

Uso:
    python3 graficar_comparacion.py               (toma el CSV más reciente de ~/ros2_ws)
    python3 graficar_comparacion.py archivo.csv

Dibuja posición y velocidad de cada articulación (Gazebo y robot real), imprime el
error de seguimiento y guarda la figura como PNG al lado del CSV.
"""
import csv
import glob
import os
import sys

import numpy as np
import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

ARTICULACIONES = [  # (columna, título, unidad, escala desde las unidades del CSV)
    ("q1", "Joint_1 (base)", "rad", 1.0),
    ("q2", "Joint_2 (codo)", "rad", 1.0),
    ("d3", "Joint_3 (eje Z)", "mm", 1000.0),
]


def suavizar(v, n=9):
    """Promedio móvil: la derivada de un encoder sale con escalones."""
    return np.convolve(v, np.ones(n) / n, mode="same")


def main():
    if len(sys.argv) > 1:
        ruta = sys.argv[1]
    else:
        archivos = glob.glob(os.path.expanduser("~/ros2_ws/robot_real_*.csv")) + glob.glob("robot_real_*.csv")
        if not archivos:
            sys.exit("No encontré ningún robot_real_*.csv (pásenle la ruta del archivo)")
        ruta = max(archivos, key=os.path.getmtime)

    with open(ruta) as f:
        filas = list(csv.DictReader(f))
    if len(filas) < 10:
        sys.exit(f"{ruta} tiene muy pocos datos ({len(filas)} filas)")

    # Tiempo del ESP32: es el instante real de cada medición (cada 20 ms)
    t = np.array([float(f["t_esp_ms"]) for f in filas]) / 1000.0
    t -= t[0]
    _, unicos = np.unique(t, return_index=True)
    filas = [filas[i] for i in unicos]
    t = t[unicos]

    fig, ejes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    print(f"Archivo: {ruta}  ({len(filas)} muestras, {t[-1]:.1f} s)\n")
    print(f"{'Articulación':<18}{'Error máximo':>16}{'Error RMS':>14}{'Error final':>14}")

    for (ax_pos, ax_vel), (k, titulo, unidad, esc) in zip(ejes, ARTICULACIONES):
        gz = np.array([float(f[f"gz_{k}"]) for f in filas]) * esc
        real = np.array([float(f[f"real_{k}"]) for f in filas]) * esc
        err = gz - real
        print(f"{titulo:<18}{np.max(np.abs(err)):>12.4f} {unidad:<3}{np.sqrt(np.mean(err**2)):>10.4f} "
              f"{unidad:<3}{abs(err[-1]):>10.4f} {unidad}")

        ax_pos.plot(t, gz, label="Gazebo", linewidth=2)
        ax_pos.plot(t, real, "--", label="Robot real", linewidth=2)
        ax_pos.set_ylabel(f"{titulo}\n[{unidad}]")
        ax_pos.grid(True, alpha=0.3)

        ax_vel.plot(t, suavizar(np.gradient(gz, t)), label="Gazebo", linewidth=2)
        ax_vel.plot(t, suavizar(np.gradient(real, t)), "--", label="Robot real", linewidth=2)
        ax_vel.set_ylabel(f"[{unidad}/s]")
        ax_vel.grid(True, alpha=0.3)

    # Momentos en que se movió la pinza (columna que agrega puente_serial.py)
    if "pinza_grados" in filas[0]:
        anterior = ""
        for ti, f in zip(t, filas):
            valor = f.get("pinza_grados") or ""
            if valor and valor != anterior:
                for fila_ejes in ejes:
                    for ax in fila_ejes:
                        ax.axvline(ti, color="gray", linestyle=":", linewidth=1)
                ejes[0][0].annotate(f"pinza {valor}°", (ti, 1.0), xycoords=("data", "axes fraction"),
                                    rotation=90, va="top", ha="right", fontsize=8, color="gray")
                print(f"  t = {ti:6.1f} s  pinza -> {valor} grados")
            anterior = valor or anterior

    ejes[0][0].set_title("Posición")
    ejes[0][1].set_title("Velocidad")
    ejes[0][0].legend()
    ejes[-1][0].set_xlabel("Tiempo [s]")
    ejes[-1][1].set_xlabel("Tiempo [s]")
    fig.suptitle("SCARA R4: simulación en Gazebo vs. robot real")
    fig.tight_layout()

    salida = os.path.splitext(ruta)[0] + ".png"
    fig.savefig(salida, dpi=150)
    print(f"\nGráfica guardada en {salida}")
    if matplotlib.get_backend().lower() != "agg":
        plt.show()


if __name__ == "__main__":
    main()
