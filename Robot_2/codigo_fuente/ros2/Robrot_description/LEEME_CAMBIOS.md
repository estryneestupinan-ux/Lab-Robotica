# Robrot: cambios y cómo usarlo (Gazebo, robot real y otra computadora)

## 1. Qué se corrigió

| Antes | Ahora |
|---|---|
| `Corredera_9`: al aumentar d3 la cremallera **subía**, y solo tenía 18 mm de recorrido | d3 positivo = la cremallera **baja**, igual que el robot real, con el mismo recorrido de **85 mm**: d3 = 0.085 es abajo del todo (home de Z) y d3 = 0 es 85 mm más arriba. |
| Servo de la pinza (`Revoluci_n_12`) giraba sobre un eje a ~30 cm del brazo del servo | El eje pasa por el centro del brazo del servo. 0 = cerrada, π/2 = abierta. |
| Dedos (`Corredera_13/14`) sin límites (±1000 m) y en sentidos distintos | 0 = cerrada, 0.018 m = abierta (cada dedo). Separación entre dedos: 11.4 mm cerrada, 47.3 mm abierta. |
| Bielas de la pinza (`Revoluci_n_15/17`) giraban libres y caían por gravedad | Fijas a cada dedo. |
| 8 joints en `ros2_control` | 6 joints: `Revoluci_n_6`, `Revoluci_n_7`, `Corredera_9`, `Revoluci_n_12`, `Corredera_13`, `Corredera_14`. |
| La pinza no se controlaba | `control_cinematico.py` controla la pinza. Al terminar **T4** la abre y la cierra **después de 2 s**; al terminar **T7**, después de **3 s**. |
| Sin conexión con el robot real | `puente_robot_real.py` conecta el mismo control con el ESP32. |

Los archivos originales quedaron en `respaldo_original/`.

Los eventos de la pinza se cambian sin tocar el código, con el parámetro `pinza_eventos`:

```bash
ros2 run Robrot_description control_cinematico.py --ros-args -p trayectoria:=todas -p pinza_eventos:="T4:2,T7:3"
```

## 2. Instalar en cualquier computadora (Linux con ROS 2 Jazzy + Gazebo Harmonic)

```bash
mkdir -p ~/ros2_ws/src
cp -r Robrot_description ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y      # instala dependencias (incluye python3-serial)
colcon build --packages-select Robrot_description
source install/setup.bash
```

El paquete no tiene rutas fijas de ninguna computadora: todo se busca con `$(find Robrot_description)`.

## 3. Simulación (Gazebo)

```bash
ros2 launch Robrot_description gazebo.launch.py
ros2 run Robrot_description control_cinematico.py --ros-args -p use_sim_time:=true -p trayectoria:=T4
ros2 run Robrot_description graficar.py ~/scara_logs/T4.csv
```

## 4. Robot real (ESP32 con `scara_verificacion.ino`)

**No** lances Gazebo al mismo tiempo: los dos publicarían `/joint_states`.

```bash
sudo usermod -aG dialout $USER      # una sola vez, para poder usar el puerto USB (cerrar sesión y volver a entrar)
ros2 run Robrot_description puente_robot_real.py --ros-args -p puerto:=/dev/ttyUSB0 -p home_al_iniciar:=true
ros2 run Robrot_description control_cinematico.py --ros-args -p trayectoria:=T4
```

Qué hace el puente:
- **Hacia el robot:** recibe `/velocity_controller/commands`, las mismas velocidades que usa Gazebo, las integra y le manda al ESP32 la posición objetivo 20 veces por segundo (comando `qq`).
- **Pinza:** recibe `/pinza/referencia` y mueve el servo real (comando `pinza`).
- **Hacia ROS:** publica la posición real de los encoders en `/joint_states`, con los nombres del URDF.

Parámetros del puente (se ajustan con `-p nombre:=valor`):

| Parámetro | Por defecto | Qué es |
|---|---|---|
| `puerto` | `/dev/ttyUSB0` | puerto del ESP32 (también acepta `socket://IP:PUERTO`) |
| `offset_hombro` | -138.11 | grados del firmware cuando `Revoluci_n_6 = 0` (sale del CAD) |
| `offset_codo` | -46.47 | grados del firmware cuando `Revoluci_n_7 = 0` (sale del CAD) |
| `z_punta_d0` | 247.0 | altura (mm) de la punta con `Corredera_9 = 0`; el robot real tiene la punta a 162 mm en el home de Z (abajo, d3 = 0.085) |
| `servo_cerrada` / `servo_abierta` | 25 / 145 | grados del servo real con la pinza cerrada / abierta (medidos; con menos de 25 el servo se fuerza) |

Con esos valores, el ESP32 y el URDF calculan la misma posición de la punta, con un error máximo de 0.05 mm en 300 posiciones de prueba. Para que además coincida con el robot físico, el **home** de cada eje en el firmware tiene que estar medido en el mismo sistema: hombro desde el eje x del mundo, codo desde el brazo estirado, Z como altura de la punta.

## 5. Trabajar desde otra computadora

**Opción A, la más simple:** conecta el USB del ESP32 a la computadora que tiene ROS y haz todo ahí.

**Opción B, dos computadoras en la misma red** (por ejemplo, PC1 con Gazebo/RViz y el control, y PC2 con el ESP32 por USB y el puente):

En **las dos** computadoras, antes de lanzar nada (o agregándolo a `~/.bashrc`):

```bash
export ROS_DOMAIN_ID=7                          # el mismo número en las dos
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET     # Jazzy (en Humble: export ROS_LOCALHOST_ONLY=0)
sudo ufw allow 7400:7700/udp                    # si el firewall está activo
```

Prueba que se ven:

```bash
ros2 run demo_nodes_cpp talker      # en PC1
ros2 run demo_nodes_cpp listener    # en PC2: debe recibir los mensajes
```

Luego:
- **PC2 (con el robot):** `puente_robot_real.py`
- **PC1:** `control_cinematico.py` y, si quieres ver el robot real en RViz, `rviz2`.

Las dos computadoras deben estar en la misma red (mismo router), y es mejor por cable o con buena señal WiFi.

## 6. Calibración del robot real (ya hecha)

| Eje | Medido |
|---|---|
| Hombro | 13.15 cuentas por grado. Switch a la izquierda (visto desde arriba): al alejarse gira en sentido horario. Recorrido de 120° (el tope derecho está a ~126°). |
| Codo | 22.2 cuentas por grado. Switch a la izquierda: al alejarse gira en sentido horario. |
| Z | 25.5 cuentas por mm. Home abajo con la punta a 162 mm de la mesa; sube hasta 85 mm. |
| Pinza | 25° cerrada, 145° abierta. |

**Pendiente:** alinear el ángulo del home del hombro y del codo con el cero del modelo, y con eso ajustar `offset_hombro` y `offset_codo`.

## 7. Trayectorias dentro del recorrido real

El hombro real solo puede ir de **q1 = -79.9 a +46.1 grados** en el modelo: +46.1 es su final de carrera y -79.9 su tope derecho.
Las trayectorias **T8 a T19** pedían hasta 80 grados, así que a todas se les restaron **40 grados en q1**. La forma del movimiento es la misma, solo girada.
Los archivos originales están en `trayectorias/originales/`.

| Trayectorias | q1 (grados) | q2 (grados) |
|---|---|---|
| T1 a T7 (sin cambios) | 0 a 45.8 | 0 a 24 |
| T8 a T19 (desplazadas) | -40 a 40.2 | 0 a 37.8 |

T7 llega a 45.8 grados, a solo 0.3 grados del final de carrera del hombro.
