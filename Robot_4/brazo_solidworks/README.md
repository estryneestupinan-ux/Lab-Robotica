# `brazo_solidworks`: paquete de ROS 2 del SCARA R4

Paquete `ament_cmake` con el modelo del robot, la simulación en Gazebo y los nodos de Python que planean los movimientos y conectan la simulación con el robot real.

## Contenido

| Carpeta / archivo | Qué contiene |
|---|---|
| `urdf/ENSAMBLAJE_CON_SUBSISTEMA.SLDASM.urdf` | Modelo del robot exportado de SolidWorks: eslabones, inercias, juntas y la sección `<ros2_control>` |
| `urdf/my_config.rviz` | Configuración de RViz para `display.launch.py` |
| `meshes/*.STL` | Mallas de `base_link`, `Link_1`, `Link_2` y `Link_3` |
| `config/controllers.yaml` | Controladores de `ros2_control` (ver abajo) |
| `launch/gazebo.launch.py` | Gazebo Sim con física `bullet-featherstone`, spawn del robot y controladores |
| `launch/display.launch.py` | Visualización en RViz con sliders (`joint_state_publisher_gui`) |
| `scripts/` | Nodos de Python, regla udev del ESP32 ([README](scripts/README.md)) |

## Juntas del modelo

| Junta | Tipo en el URDF | Eje | Rango | Interfaz de mando |
|---|---|---|---|---|
| `Joint_1` (base) | `continuous` | `0 0 1` | Sin límite en el URDF; los planeadores la mantienen en [−3.0, 3.2] rad | Posición |
| `Joint_2` (codo) | `continuous` | `0 0 1` | — | Posición |
| `Joint_3` (eje Z) | `prismatic` | `0 0 -1` | 0 (arriba) a 0.105 m (abajo) | Velocidad |

Joint_3 crece hacia **abajo**: d₃ = z₀ − z, con z₀ = 163 mm. La pose cero (todas las juntas en 0) es el brazo extendido con el eje Z arriba, igual que la pose cero del robot real.

## Controladores (`config/controllers.yaml`)

| Controlador | Tipo | Tópico de mando |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | Publica `/joint_states` |
| `joint1_position_controller` | `ForwardCommandController` (position) | `/joint1_position_controller/commands` (`Float64MultiArray`) |
| `joint2_position_controller` | `ForwardCommandController` (position) | `/joint2_position_controller/commands` (`Float64MultiArray`) |
| `joint3_velocity_controller` | `ForwardCommandController` (velocity) | `/joint3_velocity_controller/commands` (`Float64MultiArray`) |

`controller_manager` corre a 100 Hz. El eje Z se manda por velocidad y `scripts/joint3_position_hold.py` cierra encima un lazo de posición, con rampa de aceleración (tópico `/joint3_target`).

## Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select brazo_solidworks
source install/setup.bash
```

## Lanzar

```bash
ros2 launch brazo_solidworks gazebo.launch.py     # Gazebo + controladores
ros2 launch brazo_solidworks display.launch.py    # solo RViz, para revisar el modelo
```

El launch de Gazebo espera a que el robot aparezca para iniciar `joint_state_broadcaster` y, cuando este termina de cargar, inicia los tres controladores de las juntas.

Los nodos se corren con `python3 ~/ros2_ws/src/brazo_solidworks/scripts/<script>.py` o, después de compilar, con `ros2 run brazo_solidworks <script>.py`. El orden de arranque para la tarea completa está en el [instructivo de usuario](../Documentos/Instructivo_de_usuario.md).
