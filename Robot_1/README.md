# Robot 1 — Manipulador para la transferencia de aceite

<p align="center">

  <img src="./Diagramas/Ensamblaje.jpeg" alt="Robot 1 versión 2, ensamblado" height="420">

</p>

<p align="center"><em>Robot 1 (versión 2) ensamblado: estructura impresa en 3D, base cilíndrica con la electrónica a la vista y pinza en el extremo.</em></p>

Manipulador articulado de 4 grados de libertad más pinza, diseñado para tomar un recipiente con aceite, trasladarlo, inclinarlo para verter su contenido y regresar a una posición segura. Es la primera estación de una línea robotizada de producción y envasado de aceite compuesta por cinco robots.


## Contenido

1. [Estado del proyecto](#estado-del-proyecto)

2. [Función dentro de la planta](#función-dentro-de-la-planta)

3. [Especificaciones](#especificaciones)

4. [Estructura del repositorio](#estructura-del-repositorio)

5. [Flujo de trabajo](#flujo-de-trabajo)

6. [Modelo cinemático](#modelo-cinemático)

7. [Trayectoria de referencia](#trayectoria-de-referencia)

8. [Electrónica y control](#electrónica-y-control)

9. [Cómo ejecutar](#cómo-ejecutar)

10. [Seguridad](#seguridad)

11. [Documentos](#documentos)

12. [Equipo](#equipo)


## Estado del proyecto

El robot ha tenido dos versiones. La primera sirvió para validar la metodología (cinemática, trayectorias, simulación y control de motores), pero su transmisión por poleas y correa no permitía garantizar repetibilidad. La segunda reemplaza las poleas por engranajes y es la que se muestra en la foto principal.



| Aspecto | Negro, brazo largo| Rojo, compacto, base cilíndrica |

| Transmisión | Engranajes Planetarios |

| Cinemática y trayectorias | Desarrolladas y validadas en simulación | Rehechas para la nueva geometría |

| Estado |Ensamblada y puesta en funcionamiento; proceso individual ejecutado y validado mediante ROS 2 |


Los resultados de V1 se conservaron como evidencia del proceso y por eso algunos documentos de `Diagramas/` todavía se refieren a esa versiones anteriores.

<p align="center"><em>Ensamblaje completo del manipulador Robot 1.</em></p>


## Función dentro de la planta

La línea ocupa un área de 1,25 m × 1,25 m y está pensada para reducir la exposición de los operarios a tareas repetitivas y potencialmente peligrosas.

| Robot | Etapa |


| ****Robot 1**** | ****Transferencia de aceite del tanque inicial al tanque de enfriamiento**** |

| Robot 2 | Posicionamiento del envase y llenado |

| Robot 3 | Transporte del envase lleno a la estación de tapado |

| Robot 4 | Tapado |

| Robot 5 | Distribución final |

<p align="center">

  <img src="./Diagramas/Arquitectura%20del%20sistema.png" alt="Distribución de la planta" width="640">

</p>

Secuencia de operación del Robot 1:

1. Parte de la posición HOME.

2. Se aproxima al recipiente (punto A).

3. Lo sujeta con la pinza.

4. Lo eleva y lo recoge hacia la base para trasladarlo con el brazo plegado.

5. Gira y extiende hacia el punto de vertido (punto B).

6. Gira la muñeca (`joint_4`) para inclinar el recipiente y verter.

7. Deshace el giro y vuelve a HOME.


## Especificaciones

| Característica | Valor |

| Grados de libertad | 4 articulaciones rotacionales + pinza|

| Efector final | Pinza accionada por servo |

| Fabricación | Piezas impresas en 3D diseñadas en SolidWorks |

| Transmisión | Engranajes (trenes planetarios en base y codo) |

| Controlador | ESP32 con MicroPython |

| Actuadores | 3 motores DC con encoder y control PID + 2 servos |

| Posición HOME | `joint_1 = 0°`, `joint_2 = 0°`, `joint_3 = −180°`, `joint_4 = 0°` |

| Distancia de agarre | Centro de agarre a 92,67 mm del origen de `4_Link` |

| Software | ROS 2, Gazebo, RViz, MATLAB |

### Articulaciones y límites

Límites definidos en `robot2_ws/robot2_giro_135_limite_prueba.urdf`. Son provisionales y deben contrastarse con los topes mecánicos reales.

| Articulación | Función | Rango (°) |


| `joint_1` | Giro de la base (eje Z) | −180 a 180 |

| `joint_2` | Elevación del brazo | −90 a 90 |

| `joint_3` | Flexión del codo | −185 a 11,5 |

| `joint_4` | Giro de la muñeca (inclinación del recipiente) | −150 a 90 |



## Estructura del repositorio

```text

Robot_1/

├── Imagenes/             Fotos y renders del robot (V1 y V2)

├── Diagramas/            Arquitectura, cartilla e informes técnicos

├── Piezas_Robot/         Modelos CAD (STEP) y URDF exportado de SolidWorks

├── Calculos_Matlab/      Cinemática directa e inversa y generación de trayectorias

├── Trayectorias/         Trayectorias en CSV (articular y cartesiana)

├── robot2_ws/            Espacio de trabajo ROS 2: URDF, controlador, lanzadores y scripts

├── ESP32/                Firmware MicroPython del microcontrolador

├── GUI/                  Monitor gráfico de referencias articulares

├── Ejecucion_Comandos/   Guías y comandos de arranque

├── Documentacion/        Reservada para documentación adicional

└── ANALISIS_CONTEXTO.pdf Análisis del contexto de la línea robotizada

```

| Carpeta | Contenido | README |


| `Calculos_Matlab/` | Scripts de cinemática y planeación | [Ver](./Calculos_Matlab/README.md) |

| `Trayectorias/` | Archivos CSV de las trayectorias | [Ver](./Trayectorias/README.md) |

| `robot2_ws/` | Paquete ROS 2 `robot2` y scripts de ejecución | [Ver](./robot2_ws/README.md) |

| `ESP32/` | Firmware y referencia PID | [Ver](./ESP32/README.md) |

| `GUI/` | Interfaz de monitoreo | [Ver](./GUI/README.md) |

| `Ejecucion_Comandos/` | Procedimiento de arranque | [Ver](./Ejecucion_Comandos/README.md) |


## Flujo de trabajo

```text

SolidWorks ──► URDF + mallas STL ──► MATLAB (cinemática inversa y planeación)

                                          │

                                          ▼

                                   CSV de trayectoria

                                          │

                                          ▼

              ROS 2 (arm_controller) ──► Gazebo / RViz (validación)

                                          │

                                          ▼

               Puente serial USB ──► ESP32 ──► Motores y servos

```

El modelo exportado se importa en MATLAB, donde se calcula la trayectoria. El CSV resultante lo ejecuta ROS 2 sobre el controlador `arm_controller`, primero en simulación. Un puente serial reenvía las mismas referencias al ESP32 para el robot físico.

<p align="center">

  <img src="./Diagramas/Secuencia%20de%20proceso.png" alt="Diagrama de bloques del sistema" width="420">

</p>

## Modelo cinemático

Para el posicionamiento del extremo se desacopla `θ4`, que solo orienta la herramienta, y se trabaja con un modelo equivalente de 3 GDL con `d1 = 0,200 m`, `a2 = 0,150 m` y `a3 = 0,150 m` (ver [`COMPARACION_CINEMATICA_INVERSA_DIRECTA.pdf`](./Diagramas/COMPARACION_CINEMATICA_INVERSA_DIRECTA.pdf)).

****Cinemática directa****

```text

r = a2·cos(θ2) + a3·cos(θ2 + θ3)

X = r·cos(θ1)

Y = r·sin(θ1)

Z = d1 + a2·sin(θ2) + a3·sin(θ2 + θ3)

```

****Cinemática inversa**** (codo abajo)

```text

θ1 = atan2(Y, X)

r  = √(X² + Y²),   s = Z − d1

D  = (r² + s² − a2² − a3²) / (2·a2·a3)

θ3 = atan2(−√(1 − D²), D)

θ2 = atan2(s, r) − atan2(a3·sin θ3, a2 + a3·cos θ3)

```

****Jacobiano.**** Para el seguimiento cartesiano se usó control cinemático con la pseudoinversa del Jacobiano geométrico:

```text

q̇ = J†(q) · (v_ff + Kp·e)        q(k+1) = q(k) + q̇·Δt

```

En la validación de una trayectoria por Jacobiano inverso (versión 1, en simulación) el error máximo fue de unos 0,48 mm y la velocidad articular máxima de unos 8,9 °/s. Estos valores no son una validación del robot físico V2.

En MATLAB, los scripts de V2 calculan la cinemática con `inverseKinematics` sobre el URDF importado, con un `tool_tip` fijo a 92,67 mm de `4_Link`.

**---**

## Trayectoria de referencia

Trayectoria de demostración HOME → A → B → vertido → HOME.

<p align="center">
  <img src="./Diagramas/Trayectoria.png" alt="Trayectoria de movimiento del Robot 1" width="640">
</p>

<p align="center"><em>Trayectoria planificada para la secuencia HOME → A → B → vertido → HOME.</em></p>

| Fase | Movimiento | Duración |


| 1 | HOME → punto A | 8 s |

| 2 | A → B (341 muestras de MATLAB) | 17 s |

| 3 | Giro de `joint_4` de −135° (vertido) | 5 s |

| 4 | Deshacer el giro | 5 s |

| 5 | Regreso a HOME | 10 s |

Puntos de trabajo usados en MATLAB: ****A = (0,28; 0,19; 0,16) m**** y ****B = (−0,34; 0; 0,25) m****. La trayectoria incluye una elevación vertical inicial de 5 cm y un recogimiento del brazo antes de girar hacia B. MATLAB no verificó colisiones, por lo que debe revisarse en simulación antes de ejecutarla. Detalles en [`Trayectorias/`](./Trayectorias/README.md).


## Electrónica y control

| Elemento | Detalle |

| Microcontrolador | ESP32, firmware en MicroPython |

| Motores DC | Base, codo y hombro, con driver L298N |

| Realimentación | Encoder por motor (canal A; base con propuesta de cuadratura A/B) |

| Control | PID de posición por eje, a 100 Hz (`TS = 10 ms`) |

| Servos | MG996R (muñeca) y SG90 (pinza) |

| Alimentación | Regulador LM2596 y conector jack DC |

| Enlace con ROS 2 | Puerto serie USB a 115200 baudios |

Correspondencia entre articulaciones de ROS 2 y actuadores, según el protocolo `REF,seq,base,codo,hombro,muneca`:

| Articulación ROS 2 | Actuador | Pines (IN1 / IN2 / PWM / ENC) |

| `joint_1` | Motor BASE | 25 / 26 / 27 / 32 |

| `joint_2` | Motor CODO | 19 / 21 / 18 / 34 |

| `joint_3` | Motor HOMBRO | 2 / 4 / 15 / 23 |

| `joint_4` | Servo MG996R (GPIO 12) | — |

| Pinza | Servo SG90 (GPIO 13), comando independiente `GRIP,ángulo` | — |

Los nombres CODO y HOMBRO siguen la convención del firmware. Los detalles de cada versión del firmware están en [`ESP32/`](./ESP32/README.md).

## Cómo ejecutar

Los comandos usan ROS 2 **Lyrical** y suponen que el espacio de trabajo está en `\~/robot2_ws`. La guía completa, con el orden de las terminales, está en [`Ejecucion_Comandos/`](./Ejecucion_Comandos/README.md). Resumen para simulación:

```bash

# Terminal 1: Gazebo

source /opt/ros/jazzy/setup.bash && source \~/robot2_ws/install/setup.bash

gz sim -r -v 4 empty.sdf

# Terminales 2 a 4: robot_state_publisher, instanciar el robot y cargar controladores

# (ver Ejecucion_Comandos)

# Terminal de trayectoria: comprobar y luego ejecutar

python3 \~/robot2_ws/scripts/ejecutar_trayectoria.py --check

python3 \~/robot2_ws/scripts/ejecutar_trayectoria.py --run

```

El modo `--run` pide escribir `SI` entre fases.


## Seguridad

- No energizar los motores sin un ****corte físico de potencia**** accesible.

- Los firmwares del repositorio son versiones de ****prueba****: varios tienen el PWM bloqueado y los históricos tuvieron fallos de parada e inversión en codo y hombro.

- `HOME_CONFIRM` solo establece un cero lógico. Usarlo únicamente con el robot realmente en HOME e inmóvil.

- No abrir `/dev/ttyUSB0` desde dos programas a la vez.

- Los límites articulares son provisionales. Verificar los topes reales antes de cualquier prueba física.


| Documento | Descripción |



| [`ANALISIS_CONTEXTO.pdf`](./ANALISIS_CONTEXTO.pdf) | Contexto y distribución de funciones de los cinco robots |

| [`CARTILLA_SEGUNDO_CORTE.pdf`](./Diagramas/CARTILLA_SEGUNDO_CORTE.pdf) | Proceso de diseño, rediseño, modelado, simulación y validación |

| [`PLANEACION_DE_TRAYECTORIAS_PRIMERA_VERSION.pdf`](./Diagramas/PLANEACION_DE_TRAYECTORIAS_PRIMERA_VERSION.pdf) | Planeación articular con perfil trapezoidal |

| [`VALIDACION_TRAYECTORIA.pdf`](./Diagramas/VALIDACION_TRAYECTORIA.pdf) | Generación y validación de la trayectoria con ROS 2 y MATLAB |

| [`COMPARACION_CINEMATICA_INVERSA_DIRECTA.pdf`](./Diagramas/COMPARACION_CINEMATICA_INVERSA_DIRECTA.pdf) | Validación de la cinemática directa e inversa |


## Equipo

Jaider Hernández · Santiago Pinzón · Mariana Rodríguez · Kevin Ruiz

Asignatura de Robótica, Ingeniería Mecatrónica, Universidad Militar Nueva Granada. Docente: Nelson Velasco.
