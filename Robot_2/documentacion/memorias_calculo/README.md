# Memorias de cálculo

## 1. Dimensiones

| Elemento | Símbolo | Valor | Fuente |
|---|---|---|---|
| Brazo 1 (hombro–codo) | $L_1$ | 120 mm | firmware y URDF |
| Brazo 2 (codo–cremallera) | $L_2$ | 188.6 mm | firmware (el URDF da 188.3 mm) |
| Carrera vertical | $d_3$ | 0 a 85 mm | URDF y firmware |
| Altura de la punta | $z$ | 162 a 247 mm sobre la mesa | firmware |
| Alcance máximo | $L_1 + L_2$ | 308.6 mm | cálculo |
| Radio mínimo | $\lvert L_2 - L_1 \rvert$ | 68.6 mm | cálculo |

## 2. Parámetros Denavit-Hartenberg

| Articulación | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1^*$ | 0 | $L_1 = 0.120$ m | 0° |
| 2 | $\theta_2^*$ | 0 | $L_2 = 0.1886$ m | 180° |
| 3 | 0° | $d_3^*$ | 0 | 0° |

\* variable articular.

## 3. Cinemática directa

Origen sobre la mesa, en el eje del hombro. El eje $x$ apunta a lo largo de la base y los ángulos son positivos en sentido antihorario, visto desde arriba. $\theta_2$ se mide desde el brazo estirado.

$$
x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)
$$

$$
y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)
$$

$$
z = 247\ \text{mm} - d_3
$$

## 4. Cinemática inversa

$$
\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

$$
\theta_2 = \operatorname{atan2}\left(\pm\sqrt{1 - \cos^2\theta_2},\ \cos\theta_2\right)
$$

$$
\theta_1 = \operatorname{atan2}(y,\ x) - \operatorname{atan2}\left(L_2 \sin\theta_2,\ L_1 + L_2\cos\theta_2\right)
$$

$$
d_3 = 247\ \text{mm} - z
$$

El firmware prueba las dos soluciones (codo arriba y codo abajo) y se queda con la que está dentro de los límites y más cerca de la postura actual.

## 5. Jacobiano y singularidades

$$
J =
\begin{bmatrix}
-L_1 \sin\theta_1 - L_2 \sin(\theta_1+\theta_2) & -L_2 \sin(\theta_1+\theta_2) & 0 \\
L_1 \cos\theta_1 + L_2 \cos(\theta_1+\theta_2) & L_2 \cos(\theta_1+\theta_2) & 0 \\
0 & 0 & -1
\end{bmatrix}
$$

$$
\det J_{XY} = L_1 L_2 \sin\theta_2 = 0.02263\ \sin\theta_2\ \ [\text{m}^2]
$$

El Jacobiano pierde rango con $\theta_2 = 0°$ (brazo estirado) o $\theta_2 = 180°$ (brazo replegado). **$\theta_2 = 0°$ está dentro del rango del codo.** Las trayectorias probadas terminan con $\theta_2$ entre −29° (T1) y −9° (T19), así que T19 queda cerca de la singularidad ($\sin\theta_2 \approx 0.15$).

## 6. Conversión entre el modelo de ROS y el firmware

El URDF exportado del CAD tiene su propio cero articular ($q_1$, $q_2$, $d_3$). El firmware trabaja en el sistema de la sección 3. La conversión es:

$$
\theta_1 = q_1 - 138.11^\circ \qquad \theta_2 = q_2 - 46.47^\circ \qquad z\ [\text{mm}] = 247 - 1000\, d_3\ [\text{m}]
$$

Comprobación en el URDF: con $q_1 = 0$ el brazo 1 apunta a −138.11° desde el eje $x$, y con $q_2 = 0$ el brazo 2 queda a −46.6° del brazo 1. Según `LEEME_CAMBIOS.md`, el ESP32 y el URDF calculan la misma posición de la punta, con un error máximo de 0.05 mm en 300 posiciones de prueba.

| Eje | Firmware | Modelo ROS |
|---|---|---|
| Hombro | $\theta_1$ de −212° a −92° (home en −92°) | $q_1$ de −73.9° a 46.1° |
| Codo | $\theta_2$ de −92° a 88° (home en 88°) | $q_2$ de −45.5° a 134.5° |
| Vertical | $z$ de 162 a 247 mm (home en 162 mm, abajo) | $d_3$ de 0.085 (abajo) a 0 m (arriba) |

## 7. Correcciones al modelo

| Síntoma | Causa | Corrección |
|---|---|---|
| El robot aparecía acostado | El CAD se exportó con el eje Y hacia arriba | Enlace `world` con giro de 90° en X |
| Los brazos giraban lejos de la base | El exportador ubicó mal los pivotes: la junta del hombro estaba en el codo | Pivote del hombro en (0, 0) y del codo en (−89.3, −80.1) mm; mallas, inercias y juntas hijas compensadas para que la pose cero no cambie |
| Los controladores no cargaban | Dos bloques `ros2_control` con el mismo nombre y YAML anidado en `/**` | Un solo sistema y YAML plano; los spawners reciben `--param-file` |
| Juntas bloqueadas en Gazebo | El robot arrancaba fuera de sus límites | Posiciones iniciales dentro del rango y 1 mm de margen en las referencias |
| La cremallera subía con d3 positivo y solo recorría 18 mm | Eje y límites del exportador | d3 positivo baja; recorrido de 85 mm como el robot real |

## 8. Control cinemático (ROS 2)

Para cada junta $i$:

$$
u_i = \dot q_{ref,i}(t) + K_p \left( q_{ref,i}(t) - q_i \right), \qquad K_p = 2\ \text{s}^{-1}
$$

$$
\lvert u_1 \rvert,\ \lvert u_2 \rvert \le 0.20\ \text{rad/s} \qquad \lvert u_3 \rvert \le 0.15\ \text{m/s}
$$

La pinza usa la misma ley con límites de 1.5 rad/s (servo) y 0.05 m/s (dedos). Para ir al punto inicial de cada trayectoria se usa solo el término proporcional.

Las trayectorias duran 3 s y piden más velocidad de la permitida. El nodo estira el tiempo con un factor $k$ para que la referencia sea alcanzable sin cambiar su forma:

$$
k = \max\left(1,\ \max_i \frac{\lvert \dot q_{ref,i} \rvert_{\max}}{u_{\max,i}}\right) \qquad t' = k\, t \qquad \dot q'_{ref} = \frac{\dot q_{ref}}{k}
$$

| Trayectoria | $\lvert \dot q_1 \rvert$ máx. original | $k$ | Duración |
|---|---|---|---|
| T1 | 0.33 rad/s | 1.65 | 4.93 s |
| T4 | 0.43 rad/s | 2.15 | 6.43 s |
| T7 | 0.53 rad/s | 2.65 | 7.92 s |
| T8 | 0.57 rad/s | 2.83 | 8.45 s |
| T19 | 0.93 rad/s | 4.65 | 13.90 s |

El $d_3$ de los archivos (0.01 a 0.10 m) se escala linealmente al recorrido real con 1 mm de margen (0.001 a 0.084 m). Las trayectorias T8 a T19 se desplazaron −40° en $q_1$ para que quepan en el recorrido real del hombro; las originales están en `trayectorias/originales/`.

## 9. Control de bajo nivel (ESP32)

Lazo de posición por eje cada 5 ms, con PWM de 20 kHz y 10 bits (0 a 1023):

$$
PWM = \operatorname{sat}\left(k_p\, e,\ \pm PWM_{\max}\right), \qquad \lvert PWM \rvert \ge PWM_{\min}\ \text{si}\ \lvert e \rvert > tol
$$

| Eje | Resolución | $k_p$ | PWM mín. | PWM máx. | Tolerancia |
|---|---|---|---|---|---|
| Hombro | 13.15 cuentas/° | 25 por grado | 330 | 450 | 1.0° |
| Codo | 22.20 cuentas/° | 30 por grado | 300 | 700 | 0.5° |
| Eje Z | 25.50 cuentas/mm | 60 por mm | 350 | 600 | 0.5 mm |

Ajustes del lazo:

- **Zona muerta:** nunca envía menos del PWM mínimo, para vencer la fricción del arranque.
- **Histéresis:** al llegar deja de corregir hasta que el error supera dos veces la tolerancia.
- **Rampa:** al acelerar sube como máximo 15 unidades de PWM cada 5 ms; al llegar o invertir, frena de inmediato.
- **Anti-atasco:** si el encoder no avanza, sube el PWM de a poco y reinicia el driver; si sigue trabado, entra en FALLA.

Los valores están en el arreglo `CFG[3]` de `scara_verificacion.ino`.
