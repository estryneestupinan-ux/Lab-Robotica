#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Planeacion de trayectorias del robot SCARA (urdfscarafinal).

Modulo independiente de ROS: contiene la cinematica (directa e inversa),
los perfiles de interpolacion (cubico, quintico y trapezoidal/LSPB) y la
definicion de las TAREAS del robot como una lista de segmentos.

  - El nodo ROS `ejecutar_tareas.py` importa este modulo y envia los segmentos
    a los controladores de Gazebo.
  - Ejecutado solo (python3 planificador_trayectorias.py) genera las graficas
    de posicion / velocidad / aceleracion y la ruta del TCP, sin necesitar ROS.

Coordenadas: mundo de Gazebo, con el eje de la articulacion 1 en (0,0) y la
base del robot apoyada en z = 0 (ver world_joint en urdf/scara_gazebo.urdf).
"""
import numpy as np

# ---------------------------------------------------------------------------
# 1. PARAMETROS GEOMETRICOS (obtenidos del URDF exportado de SolidWorks)
# ---------------------------------------------------------------------------
V1 = np.array([0.0, -0.130])            # J1 -> J2 con q = 0   [m]
V2 = np.array([-0.016330, -0.262300])   # J2 -> TCP con q = 0  [m]
L1 = np.linalg.norm(V1)                 # 0.130 m
L2 = np.linalg.norm(V2)                 # 0.2628 m
Z_TCP0 = 0.162906                       # altura del TCP con d3 = 0  [m]

LIM = {  # limites articulares (del URDF)
    'q1': (-3.14, 3.14), 'q2': (-2.0, 2.0), 'd3': (0.0, 0.08), 'garra': (0.0, 1.0)}
VMAX = np.array([1.0, 1.0, 0.05])       # velocidad maxima deseada q1,q2 [rad/s], d3 [m/s]
VMAX_GARRA = 0.8                        # rad/s

ARM_JOINTS = ['eslabon1_joint', 'eslabon2_joint', 'prismatico_joint']
GRIPPER_JOINTS = ['garra1_joint', 'garra2_joint']

GARRA_ABIERTA = 0.70    # rad
GARRA_CERRADA = 0.20    # rad  (sujeta un cubo de 3 cm)
DT = 0.02               # periodo de muestreo de la trayectoria [s]
R_BASE = 0.19           # radio minimo del TCP con pieza (evita chocar con la base) [m]

# Geometria de la celda (debe coincidir con worlds/scara_tareas.world)
CUBO = 0.03                              # lado de los cubos [m]
H_MESA = 0.0880                          # altura de las mesas de piezas [m]
Z_AGARRE = H_MESA + CUBO / 2 + 0.0005    # altura del TCP al tomar / dejar una pieza


def _rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# 2. CINEMATICA
# ---------------------------------------------------------------------------
def cinematica_directa(q):
    """q = [q1, q2, d3]  ->  posicion del TCP [x, y, z] en el mundo."""
    q1, q2, d3 = q
    xy = _rot(q1) @ (V1 + _rot(q2) @ V2)
    return np.array([xy[0], xy[1], Z_TCP0 - d3])


def cinematica_inversa(p, q_ref=None):
    """Posicion del TCP p=[x,y,z] -> [q1,q2,d3].
    Devuelve la solucion (codo arriba/abajo) mas cercana a q_ref y valida en limites."""
    x, y, z = p
    r2 = x * x + y * y
    phi = np.arctan2(V2[1], V2[0]) - np.arctan2(V1[1], V1[0])  # angulo entre V1 y V2 en q2=0
    c = (r2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    if abs(c) > 1:
        raise ValueError('Punto (%.3f, %.3f) fuera del espacio de trabajo' % (x, y))
    d3 = Z_TCP0 - z
    if not (LIM['d3'][0] - 1e-9 <= d3 <= LIM['d3'][1] + 1e-9):
        raise ValueError('Altura z=%.3f fuera de la carrera del prismatico' % z)
    sols = []
    for s in (+1, -1):
        q2 = _wrap(s * np.arccos(c) - phi)
        w = V1 + _rot(q2) @ V2
        q1 = _wrap(np.arctan2(y, x) - np.arctan2(w[1], w[0]))
        q = np.array([q1, q2, d3])
        if LIM['q1'][0] <= q1 <= LIM['q1'][1] and LIM['q2'][0] <= q2 <= LIM['q2'][1]:
            sols.append(q)
    if not sols:
        raise ValueError('Sin solucion dentro de los limites articulares')
    if q_ref is None:
        return sols[0]
    return min(sols, key=lambda q: np.linalg.norm((q - q_ref)[:2]))


# ---------------------------------------------------------------------------
# 3. PERFILES DE INTERPOLACION  s(tau), tau in [0,1]  ->  (s, ds/dtau, d2s/dtau2)
# ---------------------------------------------------------------------------
def perfil_cubico(tau):
    return 3 * tau**2 - 2 * tau**3, 6 * tau - 6 * tau**2, 6 - 12 * tau


def perfil_quintico(tau):
    return (10 * tau**3 - 15 * tau**4 + 6 * tau**5,
            30 * tau**2 - 60 * tau**3 + 30 * tau**4,
            60 * tau - 180 * tau**2 + 120 * tau**3)


def perfil_trapezoidal(tau, ta=0.25):
    """LSPB: aceleracion constante durante ta, velocidad constante, desaceleracion."""
    v = 1.0 / (1.0 - ta)            # velocidad de crucero normalizada
    a = v / ta
    tau = np.asarray(tau, dtype=float)
    s = np.where(tau < ta, 0.5 * a * tau**2,
        np.where(tau <= 1 - ta, v * (tau - ta / 2), 1 - 0.5 * a * (1 - tau)**2))
    ds = np.where(tau < ta, a * tau, np.where(tau <= 1 - ta, v, a * (1 - tau)))
    dds = np.where(tau < ta, a, np.where(tau <= 1 - ta, 0.0, -a))
    return s, ds, dds


PERFILES = {'cubico': (perfil_cubico, 1.5),         # (funcion, pico de ds/dtau)
            'quintico': (perfil_quintico, 1.875),
            'trapezoidal': (perfil_trapezoidal, 1 / 0.75)}


def _duracion(dq, vmax, pico, t_min=1.0):
    """Duracion minima para que la velocidad pico no supere vmax."""
    return max(t_min, float(np.max(np.abs(dq) / vmax)) * pico)


def _muestras(T):
    n = max(2, int(np.ceil(T / DT)) + 1)
    return np.linspace(0.0, T, n)


def tray_articular(q0, qf, perfil='quintico', T=None, vmax=VMAX):
    """Interpolacion en el ESPACIO ARTICULAR entre q0 y qf."""
    f, pico = PERFILES[perfil]
    q0, qf = np.asarray(q0, float), np.asarray(qf, float)
    dq = qf - q0
    T = T or _duracion(dq, vmax, pico)
    t = _muestras(T)
    s, ds, dds = f(t / T)
    q = q0 + np.outer(s, dq)
    qd = np.outer(ds / T, dq)
    qdd = np.outer(dds / T**2, dq)
    return t, q, qd, qdd


def tray_cartesiana(p0, pf, q_ref, perfil='quintico', v_lin=0.05):
    """Interpolacion en LINEA RECTA del TCP (espacio cartesiano) + cinematica inversa
    en cada muestra. Velocidades y aceleraciones articulares por diferencias finitas."""
    f, pico = PERFILES[perfil]
    p0, pf = np.asarray(p0, float), np.asarray(pf, float)
    T = max(1.0, np.linalg.norm(pf - p0) / v_lin * pico)
    t = _muestras(T)
    s, _, _ = f(t / T)
    q, qa = [], q_ref
    for si in s:
        qa = cinematica_inversa(p0 + si * (pf - p0), qa)
        q.append(qa)
    q = np.array(q)
    q[:, 0] = np.unwrap(q[:, 0])
    qd = np.gradient(q, t, axis=0)
    qdd = np.gradient(qd, t, axis=0)
    qd[0] = qd[-1] = 0.0
    return t, q, qd, qdd


# ---------------------------------------------------------------------------
# 4. DEFINICION DE LAS TAREAS
# ---------------------------------------------------------------------------
HOME = np.array([0.0, 0.0, 0.0])

# Estaciones (x, y) en metros. Deben coincidir con worlds/scara_tareas.world
ESTACIONES = {
    'A1': (0.20, -0.22),   # recoger cubo rojo
    'B1': (-0.20, -0.22),  # dejar cubo rojo
    'A2': (0.28, 0.10),    # recoger cubo azul
    'B2': (-0.10, 0.28),   # dejar cubo azul
}
CUBOS = {'cubo_rojo': 'A1', 'cubo_azul': 'A2'}

# Tarea 3: recorrido rectangular cartesiano (inspeccion / soldadura simulada)
RECTANGULO = [(-0.12, -0.26), (0.12, -0.26), (0.12, -0.34), (-0.12, -0.34), (-0.12, -0.26)]
Z_RECORRIDO = 0.145


class Planificador:
    """Construye la secuencia completa de segmentos de las tareas."""

    def __init__(self, perfil='quintico'):
        self.perfil = perfil
        self.q = HOME.copy()
        self.g = 0.0
        self.segmentos = []

    # -- primitivas ---------------------------------------------------------
    def _agregar(self, tipo, datos, desc, evento=None, tarea=''):
        t, q, qd, qdd = datos
        self.segmentos.append(dict(tipo=tipo, t=t, q=q, qd=qd, qdd=qdd,
                                   desc=desc, evento=evento, tarea=tarea))

    def mover_articular(self, qf, desc, tarea=''):
        datos = tray_articular(self.q, qf, self.perfil)
        self._agregar('brazo', datos, desc, tarea=tarea)
        self.q = datos[1][-1].copy()

    def mover_a_punto(self, p, desc, tarea=''):
        self.mover_articular(cinematica_inversa(p, self.q), desc, tarea)

    def mover_lineal(self, p, desc, tarea='', v_lin=0.05):
        p0 = cinematica_directa(self.q)
        datos = tray_cartesiana(p0, p, self.q, self.perfil, v_lin)
        self._agregar('brazo', datos, desc, tarea=tarea)
        self.q = datos[1][-1].copy()

    def garra(self, ang, desc, evento=None, tarea=''):
        dq = np.array([ang - self.g])
        T = _duracion(dq, VMAX_GARRA, PERFILES[self.perfil][1], t_min=0.8)
        t, q, qd, qdd = tray_articular([self.g], [ang], self.perfil, T=T)
        # las dos garras se mueven igual (simetricas)
        self._agregar('garra', (t, np.hstack([q, q]), np.hstack([qd, qd]), np.hstack([qdd, qdd])),
                      desc, evento, tarea)
        self.g = ang

    # -- tarea compuesta: tomar y colocar ------------------------------------
    def pick_and_place(self, cubo, origen, destino, tarea):
        xa, ya = ESTACIONES[origen]
        xb, yb = ESTACIONES[destino]
        self.garra(GARRA_ABIERTA, 'Abrir garra', tarea=tarea)
        self.mover_a_punto([xa, ya, Z_TCP0], 'Aproximacion sobre %s' % origen, tarea)
        self.mover_lineal([xa, ya, Z_AGARRE], 'Descenso a %s' % origen, tarea)
        self.garra(GARRA_CERRADA, 'Cerrar garra (sujetar %s)' % cubo, ('sujetar', cubo), tarea)
        self.mover_lineal([xa, ya, Z_TCP0], 'Ascenso con pieza', tarea)
        self.mover_a_punto([xb, yb, Z_TCP0], 'Traslado a %s' % destino, tarea)
        self.mover_lineal([xb, yb, Z_AGARRE + 0.001], 'Descenso a %s' % destino, tarea)
        self.garra(GARRA_ABIERTA, 'Abrir garra (soltar %s)' % cubo, ('soltar', cubo), tarea)
        self.mover_lineal([xb, yb, Z_TCP0], 'Ascenso sin pieza', tarea)

    def recorrido_rectangular(self, tarea):
        self.garra(0.0, 'Cerrar garra', tarea=tarea)
        x0, y0 = RECTANGULO[0]
        self.mover_a_punto([x0, y0, Z_TCP0], 'Aproximacion al recorrido', tarea)
        self.mover_lineal([x0, y0, Z_RECORRIDO], 'Bajar a altura de trabajo', tarea)
        for i, (x, y) in enumerate(RECTANGULO[1:], 1):
            self.mover_lineal([x, y, Z_RECORRIDO], 'Tramo recto %d del rectangulo' % i, tarea, v_lin=0.08)
        self.mover_lineal([x0, y0, Z_TCP0], 'Subir', tarea)

    def construir(self):
        self.pick_and_place('cubo_rojo', 'A1', 'B1', 'Tarea 1: pick & place cubo rojo')
        self.pick_and_place('cubo_azul', 'A2', 'B2', 'Tarea 2: pick & place cubo azul')
        self.recorrido_rectangular('Tarea 3: trayectoria cartesiana rectangular')
        self.mover_articular(HOME, 'Regreso a HOME', 'Fin')
        return self.segmentos


def verificar(segmentos):
    """Comprueba limites articulares y que la pieza no choque con la base."""
    con_pieza = False
    for s in segmentos:
        if s['evento']:
            con_pieza = s['evento'][0] == 'sujetar'
        if s['tipo'] != 'brazo':
            continue
        q = s['q']
        assert np.all(q[:, 1] >= LIM['q2'][0]) and np.all(q[:, 1] <= LIM['q2'][1]), s['desc']
        assert np.all(q[:, 0] >= LIM['q1'][0]) and np.all(q[:, 0] <= LIM['q1'][1]), s['desc']
        assert np.all(q[:, 2] >= -1e-9) and np.all(q[:, 2] <= LIM['d3'][1] + 1e-9), s['desc']
        if con_pieza:
            r = np.array([np.linalg.norm(cinematica_directa(qi)[:2]) for qi in q])
            assert r.min() > R_BASE, 'La pieza pasa demasiado cerca de la base en: ' + s['desc']
    return True


def concatenar(segmentos):
    """Une todos los segmentos en una sola linea de tiempo (para graficar)."""
    T, Q, QD, QDD, G = [], [], [], [], []
    t0, q_brazo, g = 0.0, HOME.copy(), 0.0
    for i, s in enumerate(segmentos):
        k = 0 if i == 0 else 1          # evita instantes repetidos entre segmentos
        n = len(s['t']) - k
        T.append(t0 + s['t'][k:])
        if s['tipo'] == 'brazo':
            Q.append(s['q'][k:]); QD.append(s['qd'][k:]); QDD.append(s['qdd'][k:])
            G.append(np.full(n, g))
            q_brazo = s['q'][-1]
        else:
            Q.append(np.tile(q_brazo, (n, 1))); QD.append(np.zeros((n, 3))); QDD.append(np.zeros((n, 3)))
            G.append(s['q'][k:, 0]); g = s['q'][-1, 0]
        t0 += s['t'][-1]
    return np.concatenate(T), np.vstack(Q), np.vstack(QD), np.vstack(QDD), np.concatenate(G)


# ---------------------------------------------------------------------------
# 5. GRAFICAS (uso sin ROS)
# ---------------------------------------------------------------------------
def graficar(segmentos, carpeta='.', perfil='quintico'):
    import os
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(carpeta, exist_ok=True)
    t, q, qd, qdd, g = concatenar(segmentos)

    # limites de cada tarea para sombrear
    bandas, t0, actual = [], 0.0, None
    for s in segmentos:
        if s['tarea'] != actual:
            bandas.append([s['tarea'], t0, t0]); actual = s['tarea']
        t0 += s['t'][-1]; bandas[-1][2] = t0
    colores = ['#fde2e2', '#dbe9ff', '#e2f5e2', '#eeeeee']

    nombres = ['q1 [rad]', 'q2 [rad]', 'd3 [m]']
    fig, ax = plt.subplots(4, 3, figsize=(16, 11), sharex=True)
    for j in range(3):
        for fila, (dat, et) in enumerate([(q, 'Posicion'), (qd, 'Velocidad'), (qdd, 'Aceleracion')]):
            a = ax[fila, j]
            for k, (nom, ta, tb) in enumerate(bandas):
                a.axvspan(ta, tb, color=colores[k % 4], zorder=0)
            a.plot(t, dat[:, j], lw=1.4)
            a.set_ylabel('%s %s' % (et, nombres[j].split()[0]))
            a.grid(alpha=.3)
        ax[0, j].set_title(nombres[j])
    for k, (nom, ta, tb) in enumerate(bandas):
        ax[3, 0].axvspan(ta, tb, color=colores[k % 4], zorder=0, label=nom)
    ax[3, 0].plot(t, g, 'r', lw=1.4); ax[3, 0].set_ylabel('Garra [rad]'); ax[3, 0].grid(alpha=.3)
    ax[3, 0].legend(fontsize=7, loc='upper right')
    p = np.array([cinematica_directa(qi) for qi in q])
    ax[3, 1].plot(t, p[:, 0], label='x'); ax[3, 1].plot(t, p[:, 1], label='y'); ax[3, 1].plot(t, p[:, 2], label='z')
    ax[3, 1].set_ylabel('TCP [m]'); ax[3, 1].legend(fontsize=8); ax[3, 1].grid(alpha=.3)
    vel_tcp = np.linalg.norm(np.gradient(p, t, axis=0), axis=1)
    ax[3, 2].plot(t, vel_tcp, 'k'); ax[3, 2].set_ylabel('|v TCP| [m/s]'); ax[3, 2].grid(alpha=.3)
    for a in ax[3]:
        a.set_xlabel('t [s]')
    fig.suptitle('Planeacion de trayectorias SCARA - perfil %s (duracion total %.1f s)' % (perfil, t[-1]))
    fig.tight_layout()
    f1 = os.path.join(carpeta, 'trayectorias_articulares_%s.png' % perfil)
    fig.savefig(f1, dpi=110); plt.close(fig)

    # Ruta del TCP en planta y en 3D
    fig = plt.figure(figsize=(14, 6))
    a = fig.add_subplot(1, 2, 1)
    th = np.linspace(0, 2 * np.pi, 200)
    a.fill(np.cos(th) * (L1 + L2), np.sin(th) * (L1 + L2), color='#f3f3f3', label='Espacio de trabajo')
    a.fill(np.cos(th) * 0.153, np.sin(th) * 0.153, color='#bbbbbb', label='Base')
    a.plot(p[:, 0], p[:, 1], 'b', lw=1.2, label='Ruta del TCP')
    for k, (x, y) in ESTACIONES.items():
        a.plot(x, y, 's', ms=10, color='r' if k.endswith('1') else 'royalblue', mfc='none' if k[0] == 'B' else None)
        a.annotate(k, (x, y), xytext=(6, 6), textcoords='offset points')
    rx, ry = zip(*RECTANGULO); a.plot(rx, ry, 'g--', lw=1, label='Tarea 3 (rectangulo)')
    a.set_aspect('equal'); a.grid(alpha=.3); a.legend(fontsize=8, loc='lower left')
    a.set_xlabel('x [m]'); a.set_ylabel('y [m]'); a.set_title('Vista en planta')
    a3 = fig.add_subplot(1, 2, 2, projection='3d')
    a3.plot(p[:, 0], p[:, 1], p[:, 2], 'b', lw=1)
    a3.set_xlabel('x'); a3.set_ylabel('y'); a3.set_zlabel('z'); a3.set_title('Ruta 3D del TCP')
    fig.tight_layout()
    f2 = os.path.join(carpeta, 'ruta_tcp_%s.png' % perfil)
    fig.savefig(f2, dpi=110); plt.close(fig)
    return f1, f2


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='Genera y grafica la planeacion de trayectorias (sin ROS)')
    ap.add_argument('--perfil', default='quintico', choices=list(PERFILES))
    ap.add_argument('--salida', default='graficas')
    a = ap.parse_args()
    segs = Planificador(a.perfil).construir()
    verificar(segs)
    total = 0.0
    for s in segs:
        print('%7.2f s  %-45s %-40s %5.2f s' % (total, s['tarea'], s['desc'], s['t'][-1]))
        total += s['t'][-1]
    print('Duracion total: %.1f s' % total)
    print('Graficas:', graficar(segs, a.salida, a.perfil))
