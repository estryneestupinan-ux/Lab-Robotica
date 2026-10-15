# Robot 1 — Manipulación y vertido de aceite

Sistema robótico para la manipulación de recipientes y ejecución de operaciones de vertido en una línea automatizada de producción y envasado de aceite.

---

## Descripción del proyecto

El Robot 1 es un sistema robótico diseñado para realizar operaciones de manipulación de recipientes dentro de una línea automatizada de producción y envasado de aceite. Su función consiste en sujetar un recipiente mediante una pinza, trasladarlo hasta una posición determinada, orientar el efector final para ejecutar el vertido y regresar a una configuración segura.

El desarrollo integra el diseño mecánico, el análisis cinemático, la planificación de trayectorias y la simulación del movimiento. Se emplean herramientas de diseño asistido por computador y análisis matemático, junto con ROS 2, Gazebo y RViz, para modelar, visualizar e integrar el robot.

Además de ejecutar su tarea específica, el Robot 1 forma parte de una planta automatizada compuesta por varios robots. Por ello, la arquitectura del sistema contempla ROS 2 como infraestructura de comunicación para facilitar el intercambio de información y la coordinación entre los diferentes sistemas robóticos.

## Características del robot

| Característica | Descripción |
|---|---|
| Identificación | Robot 1 |
| Aplicación | Manipulación y vertido de aceite |
| Efector final | Pinza de agarre |
| Sistema de transmisión | Mecanismos de engranajes |
| Modelado cinemático | Cinemática directa e inversa |
| Planificación | Trayectorias articulares y cartesianas |
| Análisis matemático | Jacobiano geométrico y pseudoinversa |
| Simulación | Gazebo |
| Visualización | RViz |
| Comunicación | ROS 2 |
| Integración | Planta automatizada con múltiples robots |

## Indice
- [Diagramas](./Diagramas/)
- [Gazebo](./Gazebo/)
- [Memorias de cálculo](./Memorias%20de%20c%C3%A1lculo/)
- [Planos](./Planos/)
- [Rviz](./Rviz/)
- [Análisis de contexto](./ANALISIS_CONTEXTO.pdf)

| Elemento | Descripción |
|---|---|
| `Diagramas/` | Documentos relacionados con el desarrollo del robot. |
| `Gazebo/` | Archivos y recursos asociados a la simulación. |
| `Memorias de cálculo/` | Códigos usados durante el desarrollo del robot. |
| `Planos/` | Planos mecánicos del robot. |
| `Rviz/` | Recursos relacionados con la visualización del modelo. |
| `ANALISIS_CONTEXTO.pdf` | Documento de análisis del contexto del proyecto. |
| `README.md` | Documentación principal del Robot 1. |

## Tecnologías utilizadas

| Tecnología | Aplicación |
|---|---|
| SolidWorks | Diseño mecánico y elaboración de planos. |
| MATLAB | Análisis cinemático y planificación de trayectorias. |
| ROS 2 | Comunicación e integración de los sistemas robóticos. |
| Gazebo | Simulación del comportamiento del robot. |
| RViz | Visualización del modelo robótico. |
| URDF | Descripción de la estructura geométrica y cinemática del robot. |
| CSV | Almacenamiento e intercambio de datos de trayectorias. |
| Git y GitHub | Control de versiones y documentación. |

## Función dentro de la línea de producción

El Robot 1 participa en la etapa de manipulación y vertido de aceite. Para realizar esta operación, debe coordinar el movimiento del recipiente con la orientación de su efector final.

| Etapa | Operación |
|---|---|
| 1 | Posicionamiento inicial del robot. |
| 2 | Aproximación al recipiente. |
| 3 | Sujeción mediante la pinza. |
| 4 | Traslado hacia la posición de vertido. |
| 5 | Reorientación del efector final. |
| 6 | Ejecución de la operación de vertido. |
| 7 | Retorno a una configuración segura. |

La ejecución de esta secuencia debe contemplar la coordinación con las operaciones realizadas por los demás robots de la planta.

## Modelo cinemático

El análisis cinemático permite relacionar las variables articulares del robot con la posición y orientación de su efector final. Este modelo constituye la base para determinar las configuraciones articulares y generar trayectorias que cumplan con los requerimientos de la tarea.

### Cinemática directa

La cinemática directa obtiene la pose del efector final mediante la composición de las transformaciones entre articulaciones consecutivas. Su implementación puede representarse mediante:

```python
T = np.eye(4)

for Ti in transformaciones:
    T = T @ Ti

posicion = T[:3, 3]
orientacion = T[:3, :3]
```

### Cinemática inversa

La cinemática inversa calcula las variables articulares necesarias para alcanzar una pose objetivo. Para ello, puede emplearse un método iterativo que reduzca el error entre la posición deseada y la actual.

```python
error = posicion_objetivo - posicion_actual
q += calcular_incremento_articular(error)
```

### Jacobiano geométrico

El Jacobiano relaciona las velocidades articulares con el movimiento del efector final. Su pseudoinversa permite calcular incrementos articulares a partir del error cartesiano.

```python
J = calcular_jacobiano(q)
dq = np.linalg.pinv(J) @ error
q += dq
```
## Planificación de trayectorias

La planificación de trayectorias establece la evolución del movimiento necesario para completar las operaciones de manipulación y vertido.

El proceso comprende:

- Definición de las posiciones iniciales y finales.
- Selección de puntos intermedios para aproximación, transporte y vertido.
- Cálculo de las variables articulares necesarias.
- Generación de trayectorias articulares o cartesianas.
- Evaluación de la continuidad del movimiento y del error de seguimiento.
- Exportación de los datos de trayectoria para su utilización en el entorno de simulación.

MATLAB se utiliza para el análisis matemático y la planificación, mientras que los datos generados pueden integrarse con el modelo robótico y el entorno de simulación.

## Secuencia de operación

La operación general del Robot 1 se organiza en las siguientes fases:

1. **Posicionamiento inicial:** el robot parte de una configuración de espera o posición segura.
2. **Aproximación:** el efector final se desplaza hacia el recipiente.
3. **Agarre:** la pinza sujeta el recipiente.
4. **Traslado:** el robot mueve el recipiente hasta la región de trabajo.
5. **Orientación:** el efector final adopta la orientación necesaria para el vertido.
6. **Vertido:** se ejecuta el movimiento previsto para la operación.
7. **Retorno:** el robot regresa a una configuración segura para finalizar la secuencia o prepararse para una nueva operación.

La secuencia debe integrarse con la lógica general de la planta para facilitar la coordinación con los demás robots.

## Diseño mecánico

El diseño mecánico del Robot 1 está orientado a proporcionar los movimientos necesarios para manipular y orientar el recipiente durante la operación de vertido.

El conjunto contempla una estructura articulada, mecanismos de transmisión mediante engranajes y un efector final con pinza. La disposición de los componentes debe permitir transmitir el movimiento a las articulaciones y mantener la rigidez necesaria durante la ejecución de la tarea.

Los principales aspectos considerados son:

- Geometría y alcance del espacio de trabajo.
- Disposición de las articulaciones y sus ejes de rotación.
- Transmisión del movimiento mediante engranajes.
- Integración mecánica del efector final.
- Rigidez estructural y alineación de los elementos.
- Accesibilidad para montaje, mantenimiento y ajuste.
- Correspondencia entre el diseño mecánico y el modelo utilizado en simulación.

Los planos y las memorias de cálculo se encuentran organizados en sus respectivas carpetas dentro del repositorio.

## Simulación y visualización
### RViz

RViz permite visualizar el modelo robótico, sus articulaciones y los sistemas de referencia utilizados durante el análisis y la integración con ROS 2.
### Gazebo

Gazebo permite evaluar el comportamiento del modelo robótico en un entorno virtual. Su uso facilita analizar los movimientos previstos y verificar las trayectorias antes de realizar pruebas físicas.

Ambas herramientas complementan el análisis cinemático y permiten evaluar la correspondencia entre el modelo, las trayectorias y la configuración del sistema.

## Arquitectura del sistema

El sistema se organiza mediante una arquitectura distribuida basada en ROS 2. Esta permite separar las funcionalidades del robot en componentes de software y establecer mecanismos de comunicación para integrarlo con los demás robots de la línea de producción.

La arquitectura contempla dos niveles:

**Control del Robot 1:** comprende los componentes responsables de gestionar la tarea, procesar las trayectorias, controlar el movimiento y comunicar los estados relevantes del robot.

**Comunicación a nivel de planta:** el Robot 1 forma parte de una planta automatizada compuesta por varios robots. ROS 2 constituye la infraestructura común para intercambiar información entre los sistemas robóticos y facilitar la coordinación de sus operaciones.

| **Elemento** | **Medio de conexión** | **Función** |
|---|---|---|
| ESP32 del Robot 3 | Wi-Fi | Recibe trayectorias, controla actuadores y reporta la posición articular. |
| Router central | Red local | Conecta los controladores de los robots con los equipos de ROS 2. |
| Equipos con ROS 2 | Ethernet | Ejecutan el nodo maestro y coordinan la línea robótica. |

### Diagrama conceptual

```text
                  PLANTA AUTOMATIZADA
                           |
                   Comunicación ROS 2
                           |
             +-------------+-------------+
             |             |             |
          ROBOT 1        ROBOT 2       ROBOT N
             |
          Nodos ROS 2
             |
     +-------+--------+
     |       |        |
 Gestión  Control  Planificación
 de tarea          de trayectorias
             |
      Modelo robótico
       Gazebo / RViz
```

Los mecanismos de comunicación pueden incluir:

- **Topics:** intercambio de estados, mediciones y datos de operación.
- **Services:** solicitudes puntuales de configuración u operaciones.
- **Actions:** ejecución de tareas de mayor duración que requieren seguimiento de estado y resultados.

La selección de las interfaces depende de las necesidades de cada robot y de los nodos implementados en el sistema. El diagrama representa la arquitectura conceptual de integración de la planta.

