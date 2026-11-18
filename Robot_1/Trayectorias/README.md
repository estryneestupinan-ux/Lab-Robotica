# Trayectorias

Trayectorias calculadas en MATLAB y guardadas como CSV. Son los datos que ejecuta ROS 2 y que, a través del puente serial, llegan al ESP32.

## Archivos

| Archivo | Espacio | Muestras | Duración | Origen |
|---|---|---|---|---|
| `trayectoria_A_B_v2.csv` | Articular | 341 (cada 0,05 s) | 17 s | [`trayectorias_a_b.m`](../Calculos_Matlab/trayectorias_a_b.m) |
| `trayectoria_cartesiana.csv` | Cartesiana | 300 | 17 s | [`trayectoriatridi.m`](../Calculos_Matlab/trayectoriatridi.m) |

### `trayectoria_A_B_v2.csv`

Es la trayectoria que ejecuta `ejecutar_trayectoria.py`. Hay una copia idéntica en [`../robot2_ws/trayectorias/`](../robot2_ws/trayectorias/), que es la que leen los scripts.

| Columnas | Unidad | Descripción |
|---|---|---|
| `time_from_start` | s | Tiempo desde el inicio |
| `joint_1` … `joint_4` | rad | Posición de cada articulación |
| `velocity_1` … `velocity_4` | rad/s | Velocidad articular |
| `acceleration_1` … `acceleration_4` | rad/s² | Aceleración articular |

Extremos de la trayectoria, en grados:

| Punto | `joint_1` | `joint_2` | `joint_3` | `joint_4` | Posición cartesiana (m) |
|---|---|---|---|---|---|
| A (inicio) | −146,92 | 88,09 | −98,68 | −0,89 | (0,28; 0,19; 0,16) |
| B (final) | −0,40 | 81,93 | −85,56 | 0,14 | (−0,34; 0; 0,25) |

El recorrido incluye una elevación vertical aproximada de 5 cm desde A y un recogimiento del brazo con el codo antes de girar y extender hacia B.

### `trayectoria_cartesiana.csv`

| Columnas | Unidad | Descripción |
|---|---|---|
| `tiempo` | s | Tiempo desde el inicio |
| `X`, `Y`, `Z` | m | Posición deseada del efector final |

Va de A = (0,248; 0,201; 0,200) m a B = (−0,349; 0; 0,250) m con interpolación cúbica suave. Los puntos extremos y las alturas **no coinciden** con los de la trayectoria articular, porque provienen de otro script y otra iteración del diseño. No usar uno como referencia del otro sin recalcular.

## Uso

```bash
# Comprobar el archivo sin mover el robot simulado
python3 ~/robot2_ws/scripts/ejecutar_trayectoria.py --check

# Ejecutar la secuencia completa (pide confirmación entre fases)
python3 ~/robot2_ws/scripts/ejecutar_trayectoria.py --run

# Usar otro CSV
python3 ~/robot2_ws/scripts/ejecutar_trayectoria.py --csv ruta/al/archivo.csv --check
```

Para regenerar el CSV articular, ejecutar `trayectorias_a_b.m` en MATLAB y exportar las variables `tiempo` y `Q` con el formato de columnas anterior. El script guarda sus resultados en un `.mat`, no en CSV.

## Advertencias

- MATLAB no verificó colisiones durante la planeación.
- Los límites articulares usados al calcular la trayectoria son preliminares.
- Validar siempre en Gazebo antes de enviar la trayectoria al robot físico.
