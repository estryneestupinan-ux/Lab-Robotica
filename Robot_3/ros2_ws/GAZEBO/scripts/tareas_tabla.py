#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tarea del SCARA a partir de TU TABLA de cinematica inversa:
    HOME -> PLL -> AGARRE PLL -> PLL -> PS -> SOLTAR PS -> PS -> HOME

Planeacion en el ESPACIO ARTICULAR entre los puntos de la tabla con perfil
cubico, quintico o trapezoidal (LSPB). La duracion de cada tramo se calcula
para no superar las velocidades maximas articulares.

Marco de referencia: el mismo de la tabla (theta1 = 0 sobre el eje +X de Gazebo,
eje J1 en el origen). Theta2 de la tabla tiene el signo contrario al de Gazebo.

Uso sin ROS (solo graficas):
    python3 tareas_tabla.py --perfil quintico --salida graficas
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import planificador_trayectorias as pt  # noqa: E402  (perfiles y cinematica base)

# ---------------------------------------------------------------------------
# TABLA DE PUNTOS (tal como en tu documento)
#   alias: (x, y, z) [cm] , (theta1 [deg], theta2 [deg], d [cm])
# ---------------------------------------------------------------------------
TABLA = {
    'HOME':       ((-15, 10, 0), (93.25, -106.13, 0)),   # tu tabla dice theta1=-94.68 (no llega a (-15,10))
    'PLL':        ((0, 26, 0),   (60.07, -59.85, 0)),
    'AGARRE PLL': ((0, 26, 8),   (60.07, -59.85, 8)),
    'PS':         ((0, -26, 0),  (-119.93, -59.85, 0)),
    'SOLTAR PS':  ((0, -26, 8),  (-119.93, -59.85, 8)),
}
SIGNO_THETA2 = -1          # theta2 de la tabla tiene el signo contrario al robot

# Secuencia de la tarea: (accion, argumento, descripcion)
SECUENCIA = [
    ('mover', 'PLL',        'HOME -> PLL'),
    ('garra', 'abrir',      'Abrir garra en PLL'),
    ('mover', 'AGARRE PLL', 'PLL -> AGARRE PLL (bajar)'),
    ('garra', 'cerrar',     'Cerrar garra (sujetar cubo)'),
    ('mover', 'PLL',        'AGARRE PLL -> PLL (subir con pieza)'),
    ('mover', 'PS',         'PLL -> PS (traslado)'),
    ('mover', 'SOLTAR PS',  'PS -> SOLTAR PS (bajar)'),
    ('garra', 'abrir',      'Abrir garra (soltar cubo)'),
    ('mover', 'PS',         'SOLTAR PS -> PS (subir)'),
    ('garra', 'reposo',     'Cerrar garra en PS'),
    ('mover', 'HOME',       'PS -> HOME'),
]

# abrir <= 0.45: con mas de 0.5 rad y la cremallera arriba, el dedo 1 choca con el eslabon 2
# cerrar = 0.24: los dedos tocan el cubo de 3 cm (verificado con scripts/verificar_choques.py)
# abrir <= 0.45: con mas de 0.5 rad y la cremallera arriba, el dedo 1 choca con el eslabon 2
# cerrar = 0.24: los dedos tocan el cubo de 3 cm (verificado con scripts/verificar_choques.py)
GARRA = {'abrir': 0.45, 'cerrar': 0.24, 'reposo': 0.0}   # rad
VMAX = np.array([0.7, 0.7, 0.04])                          # rad/s, rad/s, m/s
VMAX_GARRA = 0.6

# Celda (debe coincidir con worlds/celda_tareas.sdf)
CUBO = 0.03


# CI_REAL = True: los angulos se calculan con la cinematica inversa del robot REAL
# (longitudes del CAD 13 cm y 14.8 cm + desplazamiento de la garra) para que el centro
# de la garra llegue exactamente a las coordenadas (x, y) de la tabla.
# CI_REAL = False: se usan tal cual los angulos de la tabla (modelo 15 cm / 15 cm).
CI_REAL = True
SOLO_ANGULOS = ['HOME']    # HOME es una postura de reposo: se usan los angulos de la tabla


def angulos_tabla(alias):
    t1, t2, d = TABLA[alias][1]
    return np.array([np.deg2rad(t1), SIGNO_THETA2 * np.deg2rad(t2), d / 100.0])


def cinematica_inversa_real(x, y, z_cm, q_ref):
    """Punto de la garra (x, y) [m] y bajada z [cm] -> [q1, q2, d] del robot real."""
    z = pt.Z_TCP0 - z_cm / 100.0
    return pt.cinematica_inversa(np.array([y, -x, z]), q_ref)   # mundo -> marco del CAD


def a_articular(alias):
    q_tab = angulos_tabla(alias)
    if not CI_REAL or alias in SOLO_ANGULOS:
        return q_tab
    x, y, z = TABLA[alias][0]
    return cinematica_inversa_real(x / 100.0, y / 100.0, z, q_tab)   # mismo codo que la tabla


def tcp_mundo(q):
    """Centro de la garra (entre las puntas de los dedos) en el mundo de Gazebo."""
    p = pt.cinematica_directa(q)            # marco original del CAD
    return np.array([-p[1], p[0], p[2]])     # girado 90 grados (marco de la tabla)


def eje_prismatico_mundo(q):
    """Extremo del eslabon 2 (eje del prismatico), el punto que usa la tabla."""
    v2p = np.array([-0.0096314, -0.1473])
    R = lambda a: np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    p = R(q[0]) @ (pt.V1 + R(q[1]) @ v2p)
    return np.array([-p[1], p[0], pt.Z_TCP0 + 0.0075 - q[2]])


def construir(perfil='quintico'):
    f, pico = pt.PERFILES[perfil]
    q, g = a_articular('HOME'), 0.0     # el robot arranca en HOME
    segs = []
    for accion, arg, desc in SECUENCIA:
        if accion == 'mover':
            qf = a_articular(arg)
            t, Q, QD, QDD = pt.tray_articular(q, qf, perfil, vmax=VMAX)
            segs.append(dict(tipo='brazo', t=t, q=Q, qd=QD, qdd=QDD, desc=desc,
                             destino=arg, evento=None, tarea='Pick & place PLL -> PS'))
            q = qf
        else:
            gf = GARRA[arg]
            T = pt._duracion(np.array([gf - g]), VMAX_GARRA, pico, t_min=0.8)
            t, Q, QD, QDD = pt.tray_articular([g], [gf], perfil, T=T)
            ev = None
            if 'sujetar' in desc:
                ev = ('sujetar', 'cubo')       # se ejecuta DESPUES de cerrar
            elif 'soltar' in desc:
                ev = ('soltar', 'cubo')        # se ejecuta ANTES de abrir
            segs.append(dict(tipo='garra', t=t, q=np.hstack([Q, Q]), qd=np.hstack([QD, QD]),
                             qdd=np.hstack([QDD, QDD]), desc=desc, destino=None, evento=ev,
                             tarea='Pick & place PLL -> PS'))
            g = gf
    pt.verificar(segs)
    return segs


def resumen():
    filas = []
    for alias, (xyz, ang) in TABLA.items():
        q = a_articular(alias)
        e = tcp_mundo(q) * 100 if CI_REAL else eje_prismatico_mundo(q) * 100
        filas.append((alias, xyz, ang, q, e))
    return filas


def graficar(segs, carpeta='.', perfil='quintico'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(carpeta, exist_ok=True)
    t, q, qd, qdd, g = pt.concatenar(segs)

    # instantes en que se llega a cada punto de la tabla
    marcas, t0 = [], 0.0
    for s in segs:
        t0 += s['t'][-1]
        if s['destino']:
            marcas.append((t0, s['destino']))

    fig, ax = plt.subplots(4, 3, figsize=(16, 11), sharex=True)
    nombres = [('theta1', 'deg', 180 / np.pi), ('theta2 (Gazebo)', 'deg', 180 / np.pi), ('d', 'cm', 100)]
    for j, (nom, uni, k) in enumerate(nombres):
        for fila, (dat, et, u) in enumerate([(q, 'Posicion', uni), (qd, 'Velocidad', uni + '/s'),
                                             (qdd, 'Aceleracion', uni + '/s²')]):
            a = ax[fila, j]
            a.plot(t, dat[:, j] * k, lw=1.5)
            for tm, _ in marcas:
                a.axvline(tm, color='0.8', lw=0.8, zorder=0)
            a.set_ylabel('%s [%s]' % (et, u)); a.grid(alpha=.3)
        ax[0, j].set_title(nom)
        for tm, nm in marcas:
            ax[0, j].annotate(nm, (tm, ax[0, j].get_ylim()[1]), fontsize=6, rotation=90,
                              va='top', ha='right', color='0.4')
    ax[3, 0].plot(t, g, 'r', lw=1.5); ax[3, 0].set_ylabel('Garra [rad]'); ax[3, 0].grid(alpha=.3)
    E = np.array([(tcp_mundo(qi) if CI_REAL else eje_prismatico_mundo(qi)) for qi in q]) * 100
    for c, lab in zip(range(3), 'xyz'):
        ax[3, 1].plot(t, E[:, c], label=lab)
    ax[3, 1].set_ylabel('Extremo [cm]'); ax[3, 1].legend(fontsize=8); ax[3, 1].grid(alpha=.3)
    v = np.linalg.norm(np.gradient(E, t, axis=0), axis=1)
    ax[3, 2].plot(t, v, 'k'); ax[3, 2].set_ylabel('|v extremo| [cm/s]'); ax[3, 2].grid(alpha=.3)
    for a in ax[3]:
        a.set_xlabel('t [s]')
    fig.suptitle('Trayectorias articulares HOME-PLL-PS - perfil %s (%.1f s)' % (perfil, t[-1]))
    fig.tight_layout()
    f1 = os.path.join(carpeta, 'tabla_articulares_%s.png' % perfil)
    fig.savefig(f1, dpi=110); plt.close(fig)

    fig, a = plt.subplots(figsize=(7, 7))
    th = np.linspace(0, 2 * np.pi, 200)
    a.fill(15.3 * np.cos(th), 15.3 * np.sin(th), color='#cccccc', label='Base')
    a.plot(E[:, 0], E[:, 1], 'b', lw=1.5, label='Ruta del centro de la garra' if CI_REAL else 'Ruta del eje de la cremallera')
    for alias in ('PLL', 'PS'):
        cx, cy = TABLA[alias][0][:2]
        a.add_patch(plt.Rectangle((cx - 5, cy - 5), 10, 10, fill=False, ec='g', lw=1.5))
    for alias, (xyz, ang) in TABLA.items():
        if 'AGARRE' in alias or 'SOLTAR' in alias:
            continue
        a.plot(xyz[0], xyz[1], 'r+', ms=14, mew=2)
        qa = a_articular(alias)
        e = (tcp_mundo(qa) if CI_REAL else eje_prismatico_mundo(qa)) * 100
        a.plot(e[0], e[1], 'bo', ms=6)
        a.annotate('%s\ntabla (%g, %g)\nGazebo (%.1f, %.1f)' % (alias, xyz[0], xyz[1], e[0], e[1]),
                   (xyz[0], xyz[1]), xytext=(8, -4), textcoords='offset points', fontsize=8)
    a.plot([], [], 'r+', label='Punto de la tabla'); a.plot([], [], 'bo', label='Punto alcanzado en Gazebo')
    a.set_aspect('equal'); a.grid(alpha=.3); a.legend(fontsize=8, loc='center')
    a.set_xlabel('x [cm]'); a.set_ylabel('y [cm]')
    a.set_title('Vista en planta - perfil %s' % perfil)
    fig.tight_layout()
    f2 = os.path.join(carpeta, 'tabla_planta_%s.png' % perfil)
    fig.savefig(f2, dpi=110); plt.close(fig)
    return f1, f2


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--perfil', default='quintico', choices=list(pt.PERFILES))
    ap.add_argument('--salida', default='graficas')
    a = ap.parse_args()
    print('%-11s %-14s %-24s %-28s %s' % ('ALIAS', 'tabla xyz[cm]', 'tabla th1,th2,d', 'Gazebo q1,q2[rad],d[m]', 'extremo Gazebo [cm]'))
    for alias, xyz, ang, q, e in resumen():
        print('%-11s %-14s %-24s %-28s (%.1f, %.1f)' % (alias, xyz, ang, np.round(q, 4).tolist(), e[0], e[1]))
    segs = construir(a.perfil)
    total = 0.0
    for s in segs:
        print('%6.2f s  %-34s %5.2f s' % (total, s['desc'], s['t'][-1]))
        total += s['t'][-1]
    print('Duracion total: %.1f s' % total)
    print(graficar(segs, a.salida, a.perfil))
