# robot2_ws

Espacio de trabajo de **ROS 2** del Robot 1 (versión 2). Contiene el modelo URDF, el controlador de trayectoria, los lanzadores de Gazebo y los scripts que ejecutan y validan la trayectoria A → B.

> El nombre `robot2` corresponde a la segunda versión del Robot 1. No tiene relación con el Robot_2 (SCARA) del laboratorio.

<p align="center">
  <img src="../Imagenes/robot_v2_gazebo.png" alt="Robot 1 V2 en Gazebo" width="460">
</p>

## Estructura

```text
robot2_ws/
├── robot2_giro_135_limite_prueba.urdf   URDF operativo con límites y ros2_control
├── src/robot2/                          Paquete ROS 2 (ament_cmake)
│   ├── config/controllers.yaml          Controladores
│   ├── launch/                          Lanzadores de Gazebo
│   ├── meshes/                          Mallas STL de los eslabones
│   ├── CMakeLists.txt
│   └── package.xml
├── scripts/                             Ejecución, validación y puentes con el ESP32
└── trayectorias/
    └── trayectoria_A_B_v2.csv           Trayectoria de MATLAB (copia de ../Trayectorias)
```

## Modelo del robot

`robot2_giro_135_limite_prueba.urdf` describe cinco eslabones (`base_link`, `1_Link` … `4_Link`) unidos por cuatro articulaciones rotacionales fijadas al `world`. Las mallas se cargan desde `package://robot2/meshes/`. Incluye un bloque `ros2_control` con el plugin `gz_ros2_control/GazeboSimSystem`.

| Articulación | Eje | Límites (°) |
|---|---|---|
| `joint_1` | Z | −180 a 180 |
| `joint_2` | Cercano a Y | −90 a 90 |
| `joint_3` | Cercano a X | −185 a 11,5 |
| `joint_4` | Z | −150 a 90 |

El sufijo del nombre alude a una prueba con giro de 135° de la muñeca. Los límites son provisionales.

## Controladores

`config/controllers.yaml` define el `controller_manager` a 100 Hz con dos controladores:

| Controlador | Tipo | Interfaces |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | Publica `/joint_states` |
| `arm_controller` | `JointTrajectoryController` | Comando en posición; estado en posición y velocidad. Articulaciones `joint_1` a `joint_4` |

La acción de trayectoria queda disponible en `/arm_controller/follow_joint_trajectory` y el estado en `/arm_controller/controller_state`.

## Lanzadores

Ambos suponen que el espacio de trabajo está en `~/robot2_ws`.

| Archivo | Descripción |
|---|---|
| `robot2_completo_funcional.launch.py` | Abre Gazebo, publica el modelo, instancia el robot y activa los controladores |
| `robot2_completo.launch.py` | Lo mismo y, con `demo:=true`, ejecuta automáticamente la trayectoria con `--run --auto` |

Orden de arranque: Gazebo → `robot_state_publisher` y puente del reloj → (a los 5 s) instanciar el robot → `joint_state_broadcaster` → `arm_controller` → ir a HOME.

```bash
cd ~/robot2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch robot2 robot2_completo_funcional.launch.py
ros2 launch robot2 robot2_completo.launch.py demo:=true   # con ejecución automática
```

Si se prefiere arrancar cada componente por separado, usar la guía de [`Ejecucion_Comandos`](../Ejecucion_Comandos/README.md).

## Scripts

Están escritos en Python 3 para ROS 2 y leen sus archivos desde `~/robot2_ws`.

| Script | Función |
|---|---|
| `ejecutar_trayectoria.py` | Secuencia completa HOME → A → B → giro de `joint_4` de −135° → deshacer → HOME. Opciones: `--check`, `--run`, `--auto`, `--csv` |
| `probar_AB_posiciones.py` | Envía solo el tramo A → B y pide confirmar con `SI` |
| `registrar_validacion.py` | Ejecuta la trayectoria y guarda las posiciones reales de Gazebo en `resultados/posiciones_gazebo.csv` |
| `comparar_trayectorias.py` | Compara la trayectoria de MATLAB con la registrada en Gazebo y genera gráficas en `resultados/` |
| `ros2_esp32_puente_ack.py` | Puente ROS 2 → ESP32 para **pruebas sin motores**. Envía `REF,seq,base,codo,hombro,muneca` y lee las confirmaciones |
| `puente_ros2_solo_base.py` | Puente que envía solo `joint_1` con el protocolo `BASE_REF,seq,angulo` |

`ejecutar_trayectoria.py --run` pide escribir `SI` entre fases salvo que se use `--auto`, que solo debe emplearse en simulación. `--check` valida los límites sin mover nada.

Flujo de validación:

```bash
python3 scripts/registrar_validacion.py     # ejecuta y registra
python3 scripts/comparar_trayectorias.py    # compara y grafica
```

Los puentes seriales necesitan `python3-serial` y se ejecutan **de uno en uno**, porque cada uno abre `/dev/ttyUSB0`. Pueden aceptar `--port`, `--baud` y `--hz`. El puente `puente_trayectoria_con_servos.py`, usado en el robot físico, no está en el repositorio (ver [`../GUI/`](../GUI/README.md)).

## Problemas conocidos

Detectados al revisar el contenido. Conviene corregirlos para que el espacio de trabajo se compile y arranque sin pasos manuales.

| Problema | Efecto | Solución sugerida |
|---|---|---|
| `CMakeLists.txt` instala la carpeta `urdf/`, pero no existe en `src/robot2/` | La instalación del paquete puede fallar con `colcon build` | Crear `src/robot2/urdf/` y copiar allí el URDF, o quitar `urdf` de la lista de `install(DIRECTORY ...)` |
| Los lanzadores ejecutan `scripts/ir_home.sh`, que no está en el repositorio | El último paso del arranque (ir a HOME) falla | Agregar el script o reemplazarlo por una llamada a `ejecutar_trayectoria.py` |
| Rutas fijas a `~/robot2_ws` en lanzadores y scripts | Solo funcionan si el espacio de trabajo está en esa ruta | Instalar en esa ruta o parametrizarla |
| `package.xml` tiene descripción y licencia sin completar (`TODO`) | Metadatos incompletos | Completar descripción, mantenedor y licencia |
| Faltan dependencias declaradas en `package.xml` | Los lanzadores requieren paquetes no listados (`robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `controller_manager`, `gz_ros2_control`) | Declararlas como `exec_depend` |

## Requisitos

- ROS 2 *Lyrical* (el que usan los comandos del proyecto) con Gazebo (`gz sim`), `ros_gz_sim`, `ros_gz_bridge`, `ros2_control`, `joint_trajectory_controller` y `gz_ros2_control`.
- Para los scripts de validación: `numpy` y `matplotlib`.
- Para los puentes: `python3-serial`.
