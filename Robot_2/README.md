
# Robot SCARA

## Integrantes
- Jaime Estupiñán
- Sebastián Landínez
- Andrés Bustos

## Descripción general

Este proyecto comprende el diseño, la descripción, la simulación y el control de un manipulador robótico tipo SCARA desarrollado en el Laboratorio de Robótica. El sistema busca simular el proceso de recoger y ubicar envases para su posterior llenado con aceite, como parte de una línea de producción automatizada.

## Objetivos

- Documentar la arquitectura mecánica y computacional.
- Describir el modelo cinemático y los métodos de control implementados.
- Integrar el modelo del manipulador con ROS 2.
- Validar el comportamiento mediante herramientas de visualización y simulación.
- Facilitar la instalación, ejecución y reutilización del proyecto por otros usuarios.

## Requisitos

Para utilizar el proyecto, se recomienda disponer de los siguientes componentes:

- Ubuntu con ROS 2 Jazzy instalado y configurado.
- Git para clonar el repositorio.
- `colcon` para compilar los paquetes de ROS 2.
- Gazebo para la simulación, según la configuración del proyecto.
- RViz para visualizar el modelo y sus articulaciones.

## Descargar el repositorio

Clona el repositorio del laboratorio utilizando Git:

```bash
git clone <URL_DEL_REPOSITORIO>
```

Ingresa a la carpeta del repositorio:

```bash
cd <CARPETA_DEL_REPOSITORIO>
```

El comando anterior descarga el repositorio completo del laboratorio. Dentro de él, ubica la carpeta correspondiente al proyecto SCARA.

## Preparación del entorno

Carga el entorno de ROS 2 Jazzy en la terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

El paquete `Scara_Final_description` debe estar ubicado dentro de la carpeta `src/` de un espacio de trabajo de ROS 2.

Si el espacio de trabajo ya está preparado, ingresa a su directorio raíz y compila los paquetes:

```bash
colcon build
```

Cuando la compilación finalice correctamente, carga el entorno del espacio de trabajo:

```bash
source install/setup.bash
```

**Nota:** los comandos de compilación deben ejecutarse desde la raíz del espacio de trabajo de ROS 2, no desde la carpeta del paquete.

## Ejecutar la simulación

Con ROS 2 y el espacio de trabajo configurados, ejecuta el siguiente comando para iniciar la simulación, siempre que el paquete y el archivo de lanzamiento estén instalados correctamente:

```bash
ros2 launch Scara_Final_description scara_gazebo.launch.py
```

Este comando utiliza el archivo de lanzamiento del paquete para iniciar el entorno de simulación definido para el robot.

## Arquitectura del sistema

Consultar los diagramas disponibles en `documentacion/arquitectura/` para conocer la organización mecánica y computacional del manipulador.

## Cinemática y control

En esta sección se documentan la cinemática directa e inversa, el jacobiano cuando corresponda, la planificación de trayectorias y los métodos de control utilizados para el movimiento del robot.

## Código fuente

- `codigo_fuente/ros2/`: paquetes y archivos relacionados con la descripción, simulación y control del robot mediante ROS 2.
- `codigo_fuente/microcontrolador/`: firmware y recursos asociados al control mediante microcontroladores, si forman parte de la implementación.

## Documentación técnica

Consultar la carpeta `documentacion/` para acceder a los planos mecánicos y eléctricos, diagramas, memorias de cálculo, hojas de datos y manual de usuario disponibles.

## Pruebas y resultados

Registrar los procedimientos de validación, las pruebas realizadas, los resultados obtenidos y las limitaciones identificadas durante el desarrollo y la simulación del proyecto.

## Contribuciones y reutilización

Este proyecto tiene fines académicos y busca facilitar el aprendizaje y la experimentación en robótica. Se recomienda mantener la estructura de carpetas, documentar las modificaciones realizadas y verificar los requisitos del entorno antes de ejecutar el código.

---

**Laboratorio de Robótica | Diseño, simulación y control de un manipulador SCARA.**
