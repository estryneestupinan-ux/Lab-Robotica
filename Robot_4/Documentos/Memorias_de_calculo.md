# Memorias de cálculo: SCARA R4

Cálculos que respaldan el diseño, la conversión de unidades, la planeación y el control del R4. Las ecuaciones citan la numeración de la [cartilla](Cartilla_SCARA_R4_Entrega_Final.pdf).

---

## 1. Constantes de conversión (cartilla, Ecs. 1 y 2, Tabla 4)

El ESP32 cuenta solo los flancos de subida del canal A del encoder (×1). Con N pulsos por vuelta del motor, reducción R del motorreductor, transmisión externa i y avance p del husillo:

```
k_rot = N·R·i / 2π   [cuentas/rad]          k_z = N·R·i / p   [cuentas/mm]
```

| Junta | N | R | i | p | Constante | Resolución en la junta |
|---|---|---|---|---|---|---|
| Base | 11 | 810 | 1 | — | 11·810/2π = **1418.1 cuentas/rad** | 360°/8910 = 0.040° |
| Codo | 11 | 810 | 60/20 = 3 | — | 11·810·3/2π = **4254.2 cuentas/rad** | 360°/26 730 = 0.013° |
| Eje Z | 7 | 1000 | 17/17 = 1 | 8 mm | 7·1000/8 = **875 cuentas/mm** | 1/875 mm = 1.1 µm |

**Verificación en el robot:** 90° = π/2 rad → base 1418.1·π/2 = 2227 cuentas y codo 4254.2·π/2 = 6682 cuentas; ambos giraron un cuarto de vuelta exacto. En Z, cada paso pedido de 10 mm (8750 cuentas) midió 10 mm con regla; el primer paso perdió unos 5 mm por la holgura del mecanismo.

La reducción del N20 (~1000:1) se dedujo de la prueba de motores: unas 1000 cuentas en 0.8 s a PWM 80 solo cuadran con esa reducción y 7 pulsos por vuelta.

---

## 2. Velocidades máximas físicas y límites de la Guía 1

| Junta | Cálculo | Máximo físico | Límite Guía 1 | Consigna máxima enviada | % del límite |
|---|---|---|---|---|---|
| Base | 12 rpm · 2π/60 | 1.26 rad/s | 2 rad/s | 0.40 rad/s | 20 % |
| Codo | 1.26 rad/s / 3 | 0.42 rad/s | 2 rad/s | 0.12 rad/s | 6 % |
| Eje Z | 30 rpm · 8 mm / 60 s | 4 mm/s | 150 mm/s | 3 mm/s | 2 % |

El robot cumple la Guía 1 por diseño del actuador y, además, por el limitador del puente (cartilla, Ec. 13): por cada ciclo de 20 ms, la consigna de una junta avanza como máximo v·Δt (0.008 rad en la base, 0.0024 rad en el codo y 0.06 mm en Z).

---

## 3. Cinemática (cartilla, Ecs. 4 a 8)

L₁ = 120 mm, L₂ = 160 mm, z₀ = 163 mm, d₃ ∈ [0, 105] mm.

```
x = L₁·cos q₁ + L₂·cos(q₁+q₂)          cos q₂ = (x² + y² − L₁² − L₂²) / (2·L₁·L₂)
y = L₁·sen q₁ + L₂·sen(q₁+q₂)          q₁ = atan2(y, x) − atan2(L₂·sen q₂, L₁ + L₂·cos q₂)
z = z₀ − d₃                            d₃ = z₀ − z
|J| = L₁·L₂·sen q₂,  |J|máx = 120·160 = 19 200 mm²  (q₂ = 90°)
```

**Espacio de trabajo:** |L₁ − L₂| ≤ r ≤ L₁ + L₂ → 40 mm ≤ r ≤ 280 mm.

### Poses de la tarea (cinemática directa)

| Pose | q₁ [rad] | q₂ [rad] | q₁ | q₂ | x [mm] | y [mm] | r [mm] | sen q₂ |
|---|---|---|---|---|---|---|---|---|
| Home | 3.142 | 1.800 | 180.0° | 103.1° | −83.6 | −155.8 | 176.8 | 0.97 |
| Dispensador | −1.443 | 0.918 | −82.7° | 52.6° | 153.7 | −199.2 | 251.6 | 0.79 |
| Envase | 0.391 | 0.866 | 22.4° | 49.6° | 160.3 | 197.9 | 254.7 | 0.76 |

Ejemplo (Home): x = 120·cos 180° + 160·cos 283.1° = −120 + 36.3 = −83.7 mm; y = 0 + 160·sen 283.1° = −155.8 mm.

La pose más lejana queda a 280 − 254.7 = 25.3 mm del alcance máximo, y |J| no baja del 76 % de su máximo: ninguna pose está cerca de una singularidad.

### Recta de validación (cartilla, Ec. 14)

P₀ = (200, 50, 140) mm → P₁ = (100, 200, 80) mm. Longitud: √(100² + 150² + 60²) = 190 mm (180.3 mm en el plano y 60 mm en Z).

- En T = 5 s, Z baja a 60/5 = 12 mm/s: imposible para el N20 (máximo 4 mm/s).
- En T = 25 s, Z baja a 2.4 mm/s: dentro de la capacidad del robot.

---

## 4. Planeación de la secuencia (cartilla, Ecs. 15 y 16, Tabla 11)

Perfil *smoothstep*: q(t) = q₀ + Δq·(3τ² − 2τ³), τ = t/T. Su velocidad es q̇ = Δq·6τ(1 − τ)/T, máxima en τ = 0.5: **q̇_máx = 1.5·Δq/T**. Para no pasar la velocidad vᵢ de ninguna junta:

```
T = máx( T_nom , máx_i 1.5·|Δqᵢ| / vᵢ ),    v = (0.35 rad/s, 0.11 rad/s, 2.8 mm/s)
```

| Tramo | Δq que manda | Cálculo | T |
|---|---|---|---|
| 1. Ir a Home (desde la pose cero) | Δq₂ = 1.800 rad | 1.5·1.800/0.11 | 24.5 s |
| 2. Ir al dispensador | Δq₁ = −1.443 − 3.142 = −4.585 rad | 1.5·4.585/0.35 | 19.6 s |
| 3, 4, 6, 7. Bajar o subir | Δd₃ = 30 mm | 1.5·30/2.8 | 16.1 s |
| 5. Ir al envase | Δq₁ = 0.391 + 1.443 = 1.834 rad | 1.5·1.834/0.35 | 7.9 s |
| 8. Volver a Home | Δq₂ = 1.800 − 0.866 = 0.934 rad | 1.5·0.934/0.11 | 12.7 s |

Suma de tramos: 129.1 s. Con la apertura inicial (2.8 s), las pausas de 0.5 s, la espera de asentamiento de 1 s antes de cada movimiento de pinza y el tiempo de la pinza (45°/90°/s + 0.8 s = 1.3 s cada vez), la secuencia completa suma ≈ 140 s; el ciclo que se repite (tramos 2 a 8) suma ≈ 110 s, que es lo que se midió en el robot.

**Base sin vueltas completas:** para cada objetivo q₁, se elige q₁ + 2πk (k entero) dentro de [−3.0, 3.2] rad y más cerca de la posición actual.

---

## 5. Identificación y controlador LGR (cartilla, Ecs. 9 y 10)

Modelo identificado (MATLAB, ajuste 83.53 %): G(s) = K/(1 + τs), K = 0.046814, τ = 0.002554 s.

Discretización con ZOH y Ts = 20 ms:

```
a = e^(−Ts/τ) = e^(−7.83) = 0.0004      G(z) = K·(1 − a)/(z − a) ≈ 0.04680/(z − 0.0004)
```

Como τ es unas ocho veces menor que Ts, el polo discreto queda casi en el origen: el motor se comporta como una ganancia con una muestra de retardo.

Controlador: C(z) = Kc·(z − β)/(z − 1), Kc = 6.9327, β = 0.0435 (tipo PI: polo en z = 1, cero en z = β). Ecuación característica:

```
(z − 1)(z − a) + Kc·K·(1 − a)·(z − β) = 0
z² − 0.676·z − 0.0137 = 0   →   z₁ = 0.696,  z₂ = −0.020
```

Ambos polos están dentro del círculo unitario: el lazo es estable y, por el integrador, el error de estado estacionario ante un escalón es nulo. En diferencias: u[k] = u[k−1] + 6.9327·(e[k] − 0.0435·e[k−1]).

---

## 6. Control proporcional usado en el robot (cartilla, Ec. 11, Tabla 9)

u = sign(e)·min(u_max, u_min + Kp·|e|) si |e| > e_tol; u_min = 35 vence la fricción estática.

**Equivalencia de ganancias.** Un radián del codo son tres radianes del eje de salida del motorreductor, así que Kp,codo = 3·Kp,base = 1200 PWM/rad da el mismo esfuerzo por radián de motor. La ganancia de lazo aproximada, Kp·(velocidad a PWM 80)/80, queda parecida en las tres juntas:

| Junta | Kp | Velocidad medida a PWM 80 | Ganancia de lazo |
|---|---|---|---|
| Base | 400 PWM/rad | 0.22 rad/s | 400·0.22/80 = 1.1 s⁻¹ |
| Codo | 1200 PWM/rad | 0.074 rad/s | 1200·0.074/80 = 1.1 s⁻¹ |
| Eje Z | 80 PWM/mm | 1.4 mm/s | 80·1.4/80 = 1.4 s⁻¹ |

**Error de seguimiento en rampa.** Con un lazo proporcional, seguir una consigna a velocidad v exige un error aproximadamente constante e ≈ v/(ganancia de lazo). Por eso el robot real va un poco detrás de Gazebo en los tramos rápidos y recupera la posición al final de cada tramo.

---

## 7. Servo de la pinza (cartilla, Ec. 3)

Pulso de 50 Hz (periodo de 20 000 µs), canal LEDC de 16 bits:

```
t_pulso = 500 µs + (2000 µs/180°)·θ
θ = 105° (abierta) → 1666.7 µs → 1666.7/20 000·65 535 = 5461 cuentas de PWM
θ = 60°  (cerrada) → 1166.7 µs → 3823 cuentas de PWM
```

El firmware mueve el servo a 90°/s, así que de 105° a 60° tarda 0.5 s; la secuencia espera 0.8 s más antes de soltarlo (`SERVO:OFF`).

---

## 8. Comparación modelo vs. hardware

Error máximo del modelo cinemático al reconstruir la recta: 0.017 mm. Holgura del eje Z: ≈ 3 mm. Relación: 3/0.017 ≈ 176, es decir, el error del modelo es unas 170 veces menor que la holgura mecánica: el desempeño lo limita el hardware, no el modelo.
