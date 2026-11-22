#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compara la trayectoria planeada (deseada) con la medida en Gazebo (/joint_states).
Uso:  python3 graficar_resultados.py ~/scara_resultados/deseado_quintico.csv ~/scara_resultados/real_quintico.csv"""
import os
import sys
import numpy as np


def graficar(f_des, f_real, carpeta='.', perfil=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    D = np.genfromtxt(f_des, delimiter=',', skip_header=1)
    R = np.genfromtxt(f_real, delimiter=',', skip_header=1)
    t0 = D[0, 0]
    td, tr = D[:, 0] - t0, R[:, 0] - t0
    R = R[(tr >= -0.5) & (tr <= td[-1] + 1.0)]
    tr = R[:, 0] - t0

    # la columna deseada solo tiene dato durante su propio segmento: rellenar hacia adelante
    def rellenar(col, inicial):
        out, ult = col.copy(), inicial
        for i, v in enumerate(col):
            if np.isnan(v):
                out[i] = ult
            else:
                ult = v
        return out

    nombres = ['q1 [rad]', 'q2 [rad]', 'd3 [m]', 'garra [rad]']
    fig, ax = plt.subplots(4, 2, figsize=(15, 11), sharex=True)
    for j in range(4):
        des = rellenar(D[:, 1 + j], 0.0)
        real = R[:, 1 + j]
        ax[j, 0].plot(td, des, 'b', lw=2, label='Planeada')
        ax[j, 0].plot(tr, real, 'r--', lw=1.2, label='Gazebo (real)')
        ax[j, 0].set_ylabel(nombres[j]); ax[j, 0].grid(alpha=.3)
        err = real - np.interp(tr, td, des)
        ax[j, 1].plot(tr, err, 'k', lw=1)
        ax[j, 1].set_ylabel('error ' + nombres[j].split()[0]); ax[j, 1].grid(alpha=.3)
        ax[j, 1].set_title('RMS = %.2e' % np.sqrt(np.mean(err ** 2)), fontsize=9)
    ax[0, 0].legend()
    ax[3, 0].set_xlabel('t [s]'); ax[3, 1].set_xlabel('t [s]')
    fig.suptitle('Trayectoria planeada vs ejecutada en Gazebo - perfil %s' % perfil)
    fig.tight_layout()
    out = os.path.join(carpeta, 'planeada_vs_gazebo_%s.png' % perfil)
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    print(graficar(sys.argv[1], sys.argv[2], os.path.dirname(os.path.abspath(sys.argv[1]))))
