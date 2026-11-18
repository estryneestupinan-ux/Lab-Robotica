# Ejecucion_Comandos

Guías para poner en marcha el Robot 1: simulación en Gazebo con ROS 2, ejecución de la trayectoria y conexión con el ESP32. Los comandos están escritos para **Ubuntu (WSL) con ROS 2 Lyrical** y un espacio de trabajo en `~/robot2_ws`.

## Qué leer

| Archivo | Cuándo usarlo |
|---|---|
| `COMANDOS_ACTUALES_CONFIRMADOS.txt` | Consulta rápida. Lista corta de los comandos que el equipo usa actualmente |
| `EJECUCION_PASO_A_PASO.txt` | Primera vez. Incluye restauración del espacio de trabajo, compilación, explicación de cada terminal y advertencias |
| `PROTOCOLOS_Y_VERSIONES.txt` | Antes de conectar el ESP32. Dice qué puente es compatible con qué firmware |
| `VERIFICACION.txt` | Constancia de que los `.py` compilan sintácticamente. **No** se ejecutó ROS 2, Gazebo ni el ESP32 al preparar estos archivos |

## Simulación en Gazebo

Cada comando va en una terminal distinta y todas parten de:

```bash
source /opt/ros/lyrical/setup.bash
source ~/robot2_ws/install/setup.bash
```

| Terminal | Función | Comando principal |
|---|---|---|
| 1 | Gazebo | `gz sim -r -v 4 empty.sdf` |
| 2 | Publicar el modelo | `ros2 run robot_state_publisher robot_state_publisher …` con el URDF |
| 3 | Instanciar el robot | `ros2 run ros_gz_sim create -world empty -name robot2 -file … -z 0.05` |
| 4 | Controladores | `ros2 run controller_manager spawner joint_state_broadcaster` y `arm_controller` |
| 5 | Trayectoria | `python3 ~/robot2_ws/scripts/ejecutar_trayectoria.py --check` y luego `--run` |

Antes de la terminal 1, exportar la ruta de recursos de Gazebo:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/robot2_ws/src:$HOME/robot2_ws/install/robot2/share:$GZ_SIM_RESOURCE_PATH"
```

Notas:

- Lanzar el `spawner` **una sola vez**. Si `arm_controller` ya está activo, no repetirlo.
- Verificar con `ros2 control list_controllers`.
- `--run` pide escribir `SI` entre fases.
- Para arrancar todo con un solo comando, ver los lanzadores en [`../robot2_ws/`](../robot2_ws/README.md).

Comprobaciones útiles:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action list
```

## Conexión con el ESP32 desde WSL

WSL no ve el puerto USB por defecto; hay que compartirlo con `usbipd` desde **PowerShell** (no desde la terminal de Ubuntu).

```powershell
usbipd list
usbipd attach --wsl --busid 3-2
usbipd detach --busid 3-2
```

El `BUSID` (`3-2` en las pruebas anteriores) puede cambiar: consultar el valor actual con `usbipd list`. Cerrar Thonny antes de conectar. En Ubuntu, comprobar que aparece `/dev/ttyUSB0` con `ls -l /dev/ttyUSB0`.

### Qué puente usar

| Puente (en `robot2_ws/scripts/`) | Protocolo | Firmware compatible |
|---|---|---|
| `ros2_esp32_puente_ack.py` | `REF,seq,base,codo,hombro,muneca` | `main_ros2_trayectoria_sin_potencia.py` y los que usan `REF` |
| `puente_ros2_solo_base.py` | `BASE_REF,seq,angulo` | `main_receptor_solo_base.py` |
| `puente_trayectoria_con_servos.py` | — | No está en el repositorio |

`main_base_ros2_encoder_AB.py` espera `REF,...`, pero el puente de solo base envía `BASE_REF,...`: **no son compatibles** sin modificarlos. Hay más detalle en `PROTOCOLOS_Y_VERSIONES.txt` y en el [README del firmware](../ESP32/README.md).

`--confirmar-home` (puente de prueba) envía `HOME_CONFIRM`, que fija un cero lógico. Usarlo solo si el robot está realmente en HOME, inmóvil y comprobado.

## Precauciones

- Ejecutar **un solo puente** a la vez: dos procesos no pueden abrir `/dev/ttyUSB0` juntos.
- No energizar la trayectoria física automáticamente al iniciar ROS.
- Mantener la potencia de los motores desconectada en las pruebas de comunicación.
- La trayectoria pequeña de ±10° que aparece en las guías es **solo para simulación**. No usarla en el robot físico sin verificar topes y parada de emergencia.
- No reemplazar un espacio de trabajo que funciona sin hacer antes una copia de seguridad.
- Los archivos del equipo pueden tener ediciones locales posteriores a las copias del repositorio: comparar antes de restaurar.
