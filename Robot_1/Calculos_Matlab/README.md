# Calculos_Matlab

Scripts de MATLAB para el modelado cinemático de la versión 2 del Robot 1 y la generación de sus trayectorias. Importan el URDF del robot, resuelven la cinemática inversa con `inverseKinematics` y producen las referencias articulares que luego ejecuta ROS 2.

## Requisitos

- MATLAB con **Robotics System Toolbox** (`importrobot`, `rigidBodyTree`, `inverseKinematics`).
- Un URDF con la base centrada llamado `robot2_base_centrada.urdf`, con los eslabones `base_centrada`, `1_Link` … `4_Link`. **No está incluido en el repositorio**; se obtiene a partir de [`../Piezas_Robot/robot_urdf/`](../Piezas_Robot/README.md).

## Antes de ejecutar

Todos los scripts definen la variable `archivo` con una **ruta absoluta de Windows** del equipo que los escribió. Hay que cambiarla por la ruta local del URDF en cada script, de lo contrario terminan con el error `No se encontro el archivo URDF`.

En todos los casos se agrega al final de `4_Link` un cuerpo fijo `tool_tip` desplazado 92,67 mm sobre el eje Z negativo, que representa el centro de agarre de la pinza. Esa distancia se verificó visualmente y es una aproximación respecto al CAD.

## Scripts

| Script | Qué hace |
|---|---|
| `cinematica_robot2.m` | Importa el robot, agrega el `tool_tip` y calcula la cinemática directa en la configuración inicial. Muestra el robot y el centro de agarre |
| `cinematica_inversa.m` | Cinemática inversa para el punto A. Prueba 250 configuraciones iniciales, filtra por límites articulares y prefiere la pinza horizontal |
| `cinematica_inversa_b.m` | Comprueba si el punto B es alcanzable. Prueba 1500 configuraciones, tolera hasta 10 mm de error para diagnóstico y guarda `resultado_punto_B.mat` |
| `trayectorias_a_b.m` | Trayectoria articular A → B (versión 2): elevación de 5 cm, postura intermedia de recogimiento, interpolación suave, animación 3D y gráficas. Guarda `trayectoria_A_B_v2.mat` |
| `exportar_csv.m` | Gráficas de verificación (posición, velocidad y aceleración) en tema oscuro. Usa las variables `Q` y `tiempo`, por lo que se ejecuta después de `trayectorias_a_b.m`. **No escribe ningún CSV** pese a su nombre |
| `trayectoriatridi.m` | Trayectoria cartesiana A → B de 300 puntos con interpolación cúbica `s = 3u² − 2u³`. Escribe `trayectoria_cartesiana.csv` |

## Orden recomendado

```text
1. cinematica_robot2.m        verificar el modelo y el tool_tip
2. cinematica_inversa.m       postura en A
3. cinematica_inversa_b.m     alcanzabilidad en B
4. trayectorias_a_b.m         trayectoria articular
5. exportar_csv.m             gráficas de verificación
6. trayectoriatridi.m         trayectoria cartesiana (independiente)
```

## Puntos de trabajo

| Punto | Posición (m) | Usado en |
|---|---|---|
| A | (0,28; 0,19; 0,16) | `cinematica_inversa.m`, `trayectorias_a_b.m` |
| B | (−0,34; 0; 0,25) | `cinematica_inversa_b.m`, `trayectorias_a_b.m` |
| A (cartesiana) | (0,248; 0,201; 0,200) | `trayectoriatridi.m` |
| B (cartesiana) | (−0,349; 0; 0,250) | `trayectoriatridi.m` |

El comentario de cabecera de `cinematica_inversa.m` menciona X = 30 cm, Y = 18 cm y Z = 15 cm, pero el código usa los valores de la tabla. Prevalece el código.

## Límites articulares usados

Son preliminares y difieren de los del URDF operativo en `joint_3` y `joint_4`.

| Articulación | MATLAB (°) | URDF en `robot2_ws` (°) |
|---|---|---|
| `joint_1` | −180 a 180 | −180 a 180 |
| `joint_2` | −90 a 90 | −90 a 90 |
| `joint_3` | −180 a 0 | −185 a 11,5 |
| `joint_4` | −90 a 90 | −150 a 90 |

## `referencias/`

Dos diapositivas de apoyo sobre control cinemático: actualización `q(tₖ₊₁) = q(tₖ) + J⁻¹(q)·vₑ·Δt` y planteamiento del error operacional `e = x_d − x_e`.
