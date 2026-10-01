#!/usr/bin/env python3
"""
CINEMÁTICA INVERSA - ROBOT SCARA (RRP)
Calcula trayectoria lineal, muestreo por Nyquist, cinemática inversa
y genera tablas/gráficas en Python sin dependencia de pandas.
"""

import csv
import numpy as np
import matplotlib.pyplot as plt

# 1. PARÁMETROS DEL ROBOT (mm)
L1 = 112.0      # Longitud eslabón 1
L2 = 155.0      # Longitud eslabón 2

# 2. PUNTOS DE LA TRAYECTORIA (mm)
P_inicial = np.array([170.0, 100.0, 0.0])   # [x, y, z]
P_final   = np.array([-200.0, 100.0, 0.0])

# 3. TIEMPO TOTAL Y NYQUIST
Tf = 10.0      # segundos
f0 = 1.0 / Tf  # Frecuencia fundamental [Hz]
k = 10         # Oversampling
fs = k * 2.0 * f0
dt = 1.0 / fs

print("--- Cálculo del tiempo de muestreo (Nyquist) ---")
print(f"f0 (frecuencia fundamental) = {f0:.4f} Hz")
print(f"fs (frecuencia muestreo k={k})  = {fs:.4f} Hz")
print(f"dt (tiempo de muestreo)     = {dt:.4f} s\n")

# 4. VECTOR DE TIEMPO
t = np.arange(0, Tf + dt/2.0, dt)
if t[-1] > Tf:
    t[-1] = Tf
N = len(t)
print(f"Número total de puntos: {N}\n")

# 5. INTERPOLACIÓN LINEAL
x = P_inicial[0] + (P_final[0] - P_inicial[0]) * (t / Tf)
y = P_inicial[1] + (P_final[1] - P_inicial[1]) * (t / Tf)
z = P_inicial[2] + (P_final[2] - P_inicial[2]) * (t / Tf)

# 6. ESPACIO DE TRABAJO
r = np.sqrt(x**2 + y**2)
r_max = L1 + L2
r_min = abs(L1 - L2)

if np.any(r > r_max) or np.any(r < r_min):
    raise ValueError("La trayectoria sale del espacio de trabajo del robot.")
print("Trayectoria dentro del espacio de trabajo: OK\n")

# 7. CINEMÁTICA INVERSA
cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

# Codo abajo
sin_theta2_abajo = -np.sqrt(np.maximum(0.0, 1.0 - cos_theta2**2))
theta2 = np.arctan2(sin_theta2_abajo, cos_theta2)
theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
d3 = z

theta1_deg = np.degrees(theta1)
theta2_deg = np.degrees(theta2)

# 8. IMPRESIÓN Y EXPORTACIÓN A CSV (Usando módulo nativo csv)
headers = ['t_s', 'x_mm', 'y_mm', 'z_mm', 'theta1_deg', 'theta2_deg', 'd3_mm']

# Imprimir en terminal con formato
print(f"{headers[0]:>8} {headers[1]:>8} {headers[2]:>8} {headers[3]:>8} {headers[4]:>12} {headers[5]:>12} {headers[6]:>8}")
print("-" * 72)
for i in range(N):
    print(f"{t[i]:8.4f} {x[i]:8.2f} {y[i]:8.2f} {z[i]:8.2f} {theta1_deg[i]:12.2f} {theta2_deg[i]:12.2f} {d3[i]:8.2f}")

# Guardar CSV
with open('resultados_cinematica_inversa.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(headers)
    for row in zip(t, x, y, z, theta1_deg, theta2_deg, d3):
        writer.writerow(row)

# 9. VERIFICACIÓN CINEMÁTICA DIRECTA
x_check = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
y_check = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
z_check = d3

error_max = np.max(np.sqrt((x_check - x)**2 + (y_check - y)**2 + (z_check - z)**2))
print(f"\nError máximo verificación: {error_max:.6e} mm\n")

# 10. GRAFICACIÓN
fig, axs = plt.subplots(2, 2, figsize=(11, 9))

# Subplot 1
axs[0, 0].plot(t, theta1_deg, '-o', markersize=3)
axs[0, 0].set_xlabel('Tiempo [s]')
axs[0, 0].set_ylabel(r'$\theta_1$ [grados]')
axs[0, 0].set_title(r'$\theta_1$ vs tiempo')
axs[0, 0].grid(True)

# Subplot 2
axs[0, 1].plot(t, theta2_deg, '-o', color='#d95319', markersize=3)
axs[0, 1].set_xlabel('Tiempo [s]')
axs[0, 1].set_ylabel(r'$\theta_2$ [grados]')
axs[0, 1].set_title(r'$\theta_2$ vs tiempo (codo abajo)')
axs[0, 1].grid(True)

# Subplot 3
axs[1, 0].plot(x, y, '-o', markersize=3, label='Trayectoria')
axs[1, 0].plot(P_inicial[0], P_inicial[1], 'gs', label='Inicio')
axs[1, 0].plot(P_final[0], P_final[1], 'rs', label='Fin')
axs[1, 0].set_xlabel('X [mm]')
axs[1, 0].set_ylabel('Y [mm]')
axs[1, 0].set_title('Trayectoria XY')
axs[1, 0].axis('equal')
axs[1, 0].grid(True)
axs[1, 0].legend()

# Subplot 4
idx_muestra = np.linspace(0, N - 1, 5, dtype=int)
colors = plt.cm.tab10(np.linspace(0, 1, 5))

for i, idx in enumerate(idx_muestra):
    x1 = L1 * np.cos(theta1[idx])
    y1 = L1 * np.sin(theta1[idx])
    x2 = x1 + L2 * np.cos(theta1[idx] + theta2[idx])
    y2 = y1 + L2 * np.sin(theta1[idx] + theta2[idx])
    
    axs[1, 1].plot([0, x1, x2], [0, y1, y2], '-o', color=colors[i], 
                   linewidth=2, label=f't={t[idx]:.1f}s')

axs[1, 1].plot(x, y, 'k--', label='Trayectoria')
axs[1, 1].set_xlabel('X [mm]')
axs[1, 1].set_ylabel('Y [mm]')
axs[1, 1].set_title('Configuraciones del brazo (5 instantes)')
axs[1, 1].axis('equal')
axs[1, 1].grid(True)
axs[1, 1].legend()

plt.tight_layout()
plt.show()