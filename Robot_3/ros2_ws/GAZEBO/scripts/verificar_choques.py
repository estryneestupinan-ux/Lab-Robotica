#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verifica la trayectoria planeada contra CHOQUES usando las mallas STL reales del CAD:
  - choques del robot consigo mismo (p. ej. dedo contra eslabon 2), que Gazebo no
    detecta porque la autocolision esta desactivada y los dedos no tienen colision
  - choques del robot con las mesas, el cubo y el suelo

Requiere:  pip3 install --break-system-packages trimesh python-fcl
Uso:       python3 verificar_choques.py [quintico|cubico|trapezoidal]
"""
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np

AQUI = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, AQUI)
import planificador_trayectorias as pt  # noqa: E402
import tareas_tabla as tt  # noqa: E402

try:
    import trimesh
    import fcl
except ImportError:
    sys.exit('Falta instalar:  pip3 install --break-system-packages trimesh python-fcl')

PKG = os.path.realpath(os.path.join(AQUI, '..'))
MALLAS = {'base_link': 'base_link', 'eslabon1': 'Eslabon1', 'eslabon2': 'Eslabon2',
          'prismatico': 'Prismatico', 'garra1': 'Garra1', 'garra2': 'Garra2'}
ADYACENTES = {('base_link', 'eslabon1'), ('eslabon1', 'eslabon2'), ('eslabon2', 'prismatico'),
              ('prismatico', 'garra1'), ('prismatico', 'garra2')}


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_eje(a, q):
    a = np.array(a) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def leer_urdf():
    xacro_f = os.path.join(PKG, 'urdf', 'scara_gazebo.urdf.xacro')
    txt = open(xacro_f).read().replace('$(find urdfscarafinal)', PKG)
    tmp = '/tmp/_scara_verif.xacro'
    open(tmp, 'w').write(txt)
    urdf = subprocess.run(['xacro', tmp, 'celda:=true'], capture_output=True, text=True, check=True).stdout
    juntas = []
    f = lambda s: [float(v) for v in s.split()]
    for j in ET.fromstring(urdf).findall('joint'):
        o, ax = j.find('origin'), j.find('axis')
        juntas.append(dict(nombre=j.get('name'), tipo=j.get('type'), padre=j.find('parent').get('link'),
                           hijo=j.find('child').get('link'), xyz=f(o.get('xyz')), rpy=f(o.get('rpy')),
                           eje=f(ax.get('xyz')) if ax is not None else [1, 0, 0]))
    return juntas


def cinematica(juntas, valores):
    W = {'world': np.eye(4)}
    for j in juntas:
        M = np.eye(4); M[:3, :3] = rpy(*j['rpy']); M[:3, 3] = j['xyz']
        M = W[j['padre']] @ M
        v = valores.get(j['nombre'], 0.0)
        D = np.eye(4)
        if j['tipo'] == 'prismatic':
            D[:3, 3] = np.array(j['eje']) * v
        elif j['tipo'] == 'revolute':
            D[:3, :3] = rot_eje(j['eje'], v)
        W[j['hijo']] = M @ D
    return W


def main(perfil='quintico'):
    juntas = leer_urdf()
    geo = {}
    for k, v in MALLAS.items():
        m = trimesh.load(os.path.join(PKG, 'meshes', v + '.STL'))
        b = fcl.BVHModel(); b.beginModel(len(m.vertices), len(m.faces)); b.addSubModel(m.vertices, m.faces); b.endModel()
        geo[k] = b
    caja = lambda c, s: fcl.CollisionObject(fcl.Box(*s), fcl.Transform(np.eye(3), np.array(c, float)))

    pa = tt.tcp_mundo(tt.a_articular('AGARRE PLL'))
    ps = tt.tcp_mundo(tt.a_articular('SOLTAR PS'))
    h = pa[2] - tt.CUBO / 2 - 0.0005
    fijos = {'mesa_PLL': caja((pa[0], pa[1], h / 2), (0.10, 0.10, h)),
             'mesa_PS': caja((ps[0], ps[1], h / 2), (0.10, 0.10, h)),
             'suelo': caja((0, 0, -0.05), (5, 5, 0.1))}
    cubo = np.array([pa[0], pa[1], h + tt.CUBO / 2 + 0.0005])
    nombres = pt.ARM_JOINTS + pt.GRIPPER_JOINTS

    segs = tt.construir(perfil)
    q, g, sujeto, T_rel = tt.a_articular('HOME'), 0.0, False, None
    hallazgos = {}
    for s in segs:
        ev = s['evento']
        if ev and ev[0] == 'soltar':
            sujeto = False
        for k in range(0, len(s['t']), 3):
            if s['tipo'] == 'brazo':
                q = s['q'][k]
            else:
                g = s['q'][k, 0]
            W = cinematica(juntas, dict(zip(nombres, [*q, g, g])))
            if sujeto:
                cubo = (W['prismatico'] @ T_rel)[:3]
            objs = {n: fcl.CollisionObject(geo[n], fcl.Transform(W[n][:3, :3], W[n][:3, 3])) for n in geo}
            objs.update(fijos)
            objs['cubo'] = caja(cubo, (tt.CUBO,) * 3)
            claves = list(objs)
            for i in range(len(claves)):
                for j in range(i + 1, len(claves)):
                    a, b = claves[i], claves[j]
                    if (a, b) in ADYACENTES or (b, a) in ADYACENTES:
                        continue
                    if a not in geo and b not in geo and 'cubo' not in (a, b):
                        continue
                    if {a, b} in ({'base_link', 'suelo'}, {'cubo', 'mesa_PLL'}, {'cubo', 'mesa_PS'}):
                        continue
                    if fcl.collide(objs[a], objs[b], fcl.CollisionRequest(), fcl.CollisionResult()):
                        hallazgos.setdefault((a, b), []).append(s['desc'])
        if ev and ev[0] == 'sujetar':
            sujeto = True
            W = cinematica(juntas, dict(zip(nombres, [*q, g, g])))
            T_rel = np.linalg.inv(W['prismatico']) @ np.r_[cubo, 1]

    print('\nVERIFICACION DE CHOQUES - perfil %s (%d tramos)\n' % (perfil, len(segs)))
    if not hallazgos:
        print('  Sin choques.')
    for (a, b), tramos in hallazgos.items():
        agarre = {a, b} <= {'garra1', 'garra2', 'cubo'}
        print('  %-5s %s <-> %s' % ('OK' if agarre else 'CHOQUE', a, b) +
              ('   (contacto de agarre, es normal)' if agarre else ''))
        for t in dict.fromkeys(tramos):
            print('          en: %s' % t)
    reales = [k for k in hallazgos if not set(k) <= {'garra1', 'garra2', 'cubo'}]
    print('\nResultado: %s\n' % ('SIN CHOQUES INDEBIDOS' if not reales else '%d CHOQUES A REVISAR' % len(reales)))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'quintico')
