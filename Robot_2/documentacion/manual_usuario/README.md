# Manual de usuario

## 1. Instalar ROS 2 y las dependencias

El sistema se probó con **ROS 2 Lyrical** en Ubuntu con Python 3.14. Si tu Ubuntu usa Python 3.14, no uses Jazzy: está compilado para Python 3.12 y el comando `ros2` falla al arrancar.

```bash
ls /opt/ros                               # muestra las distribuciones instaladas
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo $ROS_DISTRO                          # debe decir: lyrical

sudo apt update
sudo apt install ros-lyrical-xacro ros-lyrical-robot-state-publisher ros-lyrical-rviz2 \
                 ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
                 ros-lyrical-ros-gz ros-lyrical-gz-ros2-control \
                 python3-matplotlib python3-serial
```

No mezcles distribuciones en la misma terminal. Si aparece `ROS_DISTRO was set to 'jazzy' before`, quita de `~/.bashrc` la línea que carga Jazzy y abre una terminal nueva.

## 2. Compilar el paquete

```bash
mkdir -p ~/ros2_ws/src
cp -r <repositorio>/Robot_2/codigo_fuente/ros2/Robrot_description ~/ros2_ws/src/
cd ~/ros2_ws                              # compila SIEMPRE desde la raíz, no desde src
colcon build --packages-select Robrot_description
source install/setup.bash
ls $(ros2 pkg prefix Robrot_description)/lib/Robrot_description
# debe listar: control_cinematico.py  graficar.py  puente_robot_real.py
```

Si tienes otra copia de `Robrot_description` en `src/`, sácala del espacio de trabajo: colcon no acepta dos paquetes con el mismo nombre. Los avisos de *naming conventions* por las mayúsculas del nombre se pueden ignorar.

## 3. Ver el modelo en RViz (sin física)

```bash
ros2 launch Robrot_description display.launch.py
```

Usa hardware simulado (mock) y `position_controller`. En **Global Options → Fixed Frame** debe decir `world`.

## 4. Simulación en Gazebo

**Terminal 1:**

```bash
ros2 launch Robrot_description gazebo.launch.py
```

**Terminal 2:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
# joint_state_broadcaster y velocity_controller deben estar "active"
ros2 run Robrot_description control_cinematico.py --ros-args -p use_sim_time:=true -p trayectoria:=T4
```

Parámetros útiles de `control_cinematico.py`:

| Parámetro | Por defecto | Uso |
|---|---|---|
| `trayectoria` | `T1` | Nombre del archivo (`T1`, `T4`, `T7`, `T8`, `T19`…) o `todas` |
| `pinza_eventos` | `"T4:2,T7:3"` | Trayectorias tras las que la pinza abre, y segundos que queda abierta |
| `kp` | 2.0 | Ganancia proporcional [1/s] |
| `w_max` / `v_max` | 0.20 / 0.15 | Límites de velocidad [rad/s] y [m/s] |
| `escalar_tiempo` | `true` | Estira la trayectoria para respetar los límites; con `false` solo satura |
| `use_sim_time` | `false` | `true` con Gazebo |

**Gráficas:** cada trayectoria deja un registro en `~/scara_logs/`.

```bash
ros2 run Robrot_description graficar.py ~/scara_logs/T4.csv
```

## 5. Robot físico copiando a Gazebo (modo usado en las pruebas)

### Una sola vez

```bash
sudo usermod -aG dialout $USER            # luego cierra sesión y vuelve a entrar
ls /dev/ttyUSB* /dev/ttyACM*              # con el ESP32 conectado, para saber el puerto
```

El firmware `scara_verificacion.ino` debe estar cargado y calibrado (ver la [guía del firmware](../../codigo_fuente/microcontrolador/scara_verificacion/GUIA_VERIFICACION.md)).

### Antes de cada sesión

- Fuente de **24 V** encendida y finales de carrera conectados.
- Monitor serie de Arduino y visor web **cerrados**: el puerto solo lo puede usar un programa.
- Nada en el camino del robot y la mano cerca del botón **BOOT** del ESP32.

### Orden de arranque

**Terminal 1: Gazebo**

```bash
ros2 launch Robrot_description gazebo.launch.py
```

**Terminal 2: espejo hacia el robot**

```bash
source ~/ros2_ws/install/setup.bash
python3 <repositorio>/Robot_2/codigo_fuente/ros2/herramientas/espejo_gazebo_real.py --ros-args -p puerto:=/dev/ttyUSB0
```

Espera a que termine el home (Z, codo y hombro) y aparezca `Siguiendo a Gazebo.`. Desde ese momento el robot se acomoda a la pose de Gazebo.

**Terminal 3: control**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run Robrot_description control_cinematico.py --ros-args \
  -p use_sim_time:=true -p trayectoria:=T4 -p pinza_eventos:="T4:2,T7:3"
```

Cambia `T4` por `T1`, `T7`, `T8` o `T19` para las demás trayectorias.

### Apagar

`Ctrl + C` primero en la terminal 3, luego en la 2 (envía `s` al ESP32) y al final en la 1.

### Parámetros del espejo

| Parámetro | Por defecto | Uso |
|---|---|---|
| `puerto` | `/dev/ttyUSB0` | Puerto del ESP32 |
| `home_al_iniciar` | `true` | Hace home al conectar (el ESP32 lo pierde al reiniciarse) |
| `offset_hombro` / `offset_codo` | −138.11 / −46.47 | Grados del firmware con q1 = 0 y q2 = 0 |
| `z_punta_d0` | 247.0 | Altura de la punta [mm] con d3 = 0 |
| `servo_cerrada` / `servo_abierta` | 25 / 145 | Grados del servo real |
| `w_max` / `v_max` | 0.20 / 0.15 | Velocidad máxima del objetivo enviado al robot |

**Calibrar el cero:** con Gazebo recién abierto (q1 = q2 = 0), compara desde arriba el robot real con Gazebo. Si el brazo 1 real queda Δ grados más antihorario, resta Δ a `offset_hombro`; si queda más horario, súmalo. Lo mismo para el codo con `offset_codo`.

## 6. Robot físico sin Gazebo (alternativa)

`puente_robot_real.py` mueve el robot real con el mismo control, sin simulación. **No abras Gazebo al mismo tiempo**: los dos publicarían `/joint_states`.

```bash
ros2 run Robrot_description puente_robot_real.py --ros-args -p puerto:=/dev/ttyUSB0 -p home_al_iniciar:=true
ros2 run Robrot_description control_cinematico.py --ros-args -p trayectoria:=T4
```

Para trabajar con dos computadores en la misma red, mira la sección 5 de [LEEME_CAMBIOS.md](../../codigo_fuente/ros2/Robrot_description/LEEME_CAMBIOS.md). Las dos máquinas deben tener la misma distribución de ROS 2.

## 7. Solución de problemas

| Mensaje o síntoma | Causa | Solución |
|---|---|---|
| `Could not find a package configuration file provided by "ament_cmake"` | La terminal no tiene ROS cargado | `source /opt/ros/lyrical/setup.bash` y compilar desde `~/ros2_ws` |
| `No module named 'rclpy._rclpy_pybind11'` | Se cargó Jazzy en un sistema con Python 3.14 | Usar Lyrical; borrar `build install log` y recompilar |
| `Unable to parse the value of parameter robot_description as yaml` | Launch antiguo para esta versión de ROS | Usar los launch de este paquete (envuelven el URDF en `ParameterValue`) |
| `parameter 'joints' is not initialized` | El controlador no recibe su YAML | Usar los launch de este paquete (pasan `--param-file`) |
| Piezas blancas y regadas en RViz | No llega `/joint_states` | Revisar con `ros2 control list_controllers` que el broadcaster esté activo |
| `Command of at least one joint is out of limits` | Una junta está fuera de su rango | Usar los límites y posiciones iniciales de este paquete |
| El control se queda en «Yendo al punto inicial» | Una junta no puede llegar a su objetivo | Revisar los mensajes de error por junta que imprime el nodo |
| `Permission denied: '/dev/ttyUSB0'` | El usuario no está en `dialout` | `sudo usermod -aG dialout $USER` y volver a iniciar sesión; por ahora, `sudo chmod a+rw /dev/ttyUSB0` |
| El espejo muestra `rst:0x1 (POWERON_RESET)` | Abrir el USB reinicia el ESP32 | Normal; el espejo espera 3 s y repite el home |
| `Esperando el estado del ESP32 (líneas @)` | Puerto equivocado u ocupado | Cerrar el monitor serie o el visor; revisar el nombre del puerto |
| `!!! FALLA` en el ESP32 | Eje trabado, encoder sin contar o final de carrera no encontrado | Corregir la causa y reiniciar el espejo (manda `reset` y `home`) |
| Carpetas sin permisos al descomprimir un .zip hecho en Windows | El zip guarda permisos vacíos | `chmod -R u+rwX,go+rX <carpeta>` |
