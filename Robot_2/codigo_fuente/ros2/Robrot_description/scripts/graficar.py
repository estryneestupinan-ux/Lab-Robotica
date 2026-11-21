#!/usr/bin/env python3
"""Grafica un log de control cinemático: referencia vs real y velocidades con sus límites.
Uso:  ros2 run Robrot_description graficar.py ~/scara_logs/T1.csv
"""
import sys
import numpy as np
import matplotlib.pyplot as plt

f = sys.argv[1]
a = np.genfromtxt(f, delimiter=',', names=True)
fig, ax = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
nom = [('q1', 'rad', 'u1_rad_s', 0.20), ('q2', 'rad', 'u2_rad_s', 0.20), ('d3', 'm', 'u3_m_s', 0.15)]
for i, (n, un, u, lim) in enumerate(nom):
    ax[i, 0].plot(a['t'], a[n + '_ref'], '--', label='referencia')
    ax[i, 0].plot(a['t'], a[n], label='real (Gazebo)')
    ax[i, 0].set_ylabel(f'{n} [{un}]'); ax[i, 0].grid(True); ax[i, 0].legend()
    ax[i, 1].plot(a['t'], a[u])
    ax[i, 1].axhline(lim, color='r', ls=':'); ax[i, 1].axhline(-lim, color='r', ls=':')
    ax[i, 1].set_ylabel(f'u{i+1} [{un}/s]'); ax[i, 1].grid(True)
ax[2, 0].set_xlabel('t [s]'); ax[2, 1].set_xlabel('t [s]')
fig.suptitle(f'Control cinemático SCARA RRP — {f.split("/")[-1]}')
fig.tight_layout()
out = f.replace('.csv', '.png'); fig.savefig(out, dpi=120); print('Guardado', out)
plt.show()
