# Instructivo de usuario: SCARA R4

Cómo poner en marcha el R4, ejecutar la tarea de colocación de tapas y resolver los problemas más comunes. Versión de firmware: **2026-11-18f**.

> ⚠️ **Seguridad.** Las protecciones automáticas de atasco y de error grande están desactivadas: si algo falla, el robot **no se detiene solo**. Para detenerlo en cualquier momento: **Ctrl+C en la terminal del puente** (terminal 3). El watchdog del ESP32 apaga los motores 500 ms después de que dejan de llegar consignas. Mantener las manos fuera del radio del brazo (280 mm) mientras la secuencia corre.

---

## 1. Preparación (solo la primera vez)

1. Instalar dependencias y compilar el paquete, como indica el [README principal](../README.md#-instalación).
2. Instalar la regla udev para que el ESP32 aparezca como `/dev/esp32`:
   ```bash
   sudo cp ~/ros2_ws/src/brazo_solidworks/scripts/99-esp32.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. Grabar el firmware desde `firmware_esp32/` con `pio run -t upload` (con el conector U6 de la base desconectado, por el GPIO 12).

---

## 2. Conexiones (con la fuente de 12 V apagada y sin USB)

| Conector | Elemento |
|---|---|
| U5 | M1, eje Z (motor pequeño N20) |
| U4 | M2, codo (JGB37 con correa) |
| U6 | M3, base (JGB37 de abajo) |
| H1 | Servo de la pinza: pin 1 señal (GPIO 23), pin 2 +5 V, pin 3 GND |

- Orden de pines de U4, U5 y U6: 1 = encoder A, 2 = encoder B, 3 = 5 V, 4 = GND, 5 y 6 = motor.
- **Un conector al revés mete los 12 V del motor al ESP32.** Revisar la orientación antes de encender.
- El servo va con su propia fuente de 5 V, con la tierra unida a la de la placa.
- **Encender la fuente de 12 V antes de probar.** Sin 12 V los motores no giran, y el servo puede reiniciar el ESP32 (`Brownout`) porque todo queda alimentado por el USB.
- Si al conectar el USB el ESP32 no arranca, desconectar U6 (base), conectar el USB y volver a conectar U6.

---

## 3. Revisar el ESP32 (monitor serial)

```bash
python3 -m serial.tools.miniterm /dev/esp32 115200 --echo --exit-char 3
```

Escribir **un comando a la vez** y esperar la respuesta:

| Comando | Qué debe pasar |
|---|---|
| `INFO` | La primera línea dice `MSG:firmware 2026-11-18f` y la segunda termina en `proteccion_atasco=OFF proteccion_error=OFF` |
| `PRUEBA` | Tres `OK`: Z ≈ 950 cuentas, codo ≈ 250, base ≈ 250 |
| `SERVO:105` y luego `SERVO:60` | La pinza abre y cierra **sin** que aparezca `Brownout` |
| `SERVO:OFF` | El servo queda sin pulsos |
| `ZERO` | Con el robot puesto a mano en la pose cero: **brazo extendido y eje Z arriba** |

Salir del monitor con **Ctrl+C**: el puerto lo puede usar un solo programa a la vez.

---

## 4. Limpiar procesos anteriores

```bash
pkill -9 -f "gz sim"; pkill -9 -f "ruby"; pkill -9 -f "controller_manager"; pkill -9 -f "joint3_position_hold"; pkill -9 -f "joint_angle_gui"
pkill -f "puente_serial"; pkill -f "secuencia_gui"; pkill -f "miniterm"; pkill -f "device monitor"
sleep 2
```

---

## 5. Arrancar (cuatro terminales)

**Terminal 1: Gazebo**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch brazo_solidworks gazebo.launch.py
```
Esperar a que abra Gazebo y la terminal confirme que se cargaron los controladores.

**Terminal 2: control del eje Z en Gazebo**
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/brazo_solidworks/scripts/joint3_position_hold.py
```

**Terminal 3: puente al robot real** (omitir para trabajar solo en simulación)
```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/brazo_solidworks/scripts
python3 puente_serial.py
```
Esperar a que aparezcan `Robot real en: ... 0.000` y `Conectado a Gazebo: ... 0.000`.

**Terminal 4: secuencia**
```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/brazo_solidworks/scripts
python3 secuencia_gui.py
```
No abrir `joint_angle_gui.py` al mismo tiempo: los dos programas mandan consignas a las mismas juntas.

---

## 6. Ejecutar la tarea

1. En la ventana, probar **Abrir** y **Cerrar**. En la terminal 3 aparece `Pinza -> 105 grados` y, un segundo después, `Pinza suelta (servo sin pulsos)`.
2. Colocar una tapa en el dispensador y el envase en su lugar.
3. Presionar **Iniciar Secuencia**. La secuencia dura unos 2 min 20 s la primera vez (unos 110 s por ciclo de Home a Home):
   1. Abre la pinza.
   2. Va a Home despacio (~25 s).
   3. Va al dispensador, baja, **cierra la pinza** y sube con la tapa.
   4. Va al envase, baja, **abre la pinza** y sube.
   5. Vuelve a Home.
4. La etiqueta de la ventana indica el tramo en curso y cuándo espera al robot real.
5. Al terminar, Ctrl+C en la terminal 3 y graficar la comparación:
   ```bash
   python3 graficar_comparacion.py
   ```

---

## 7. Ajustes sin volver a grabar el ESP32

Cerrar la ventana de la secuencia, cambiar el valor y volver a abrirla.

```bash
cd ~/ros2_ws/src/brazo_solidworks/scripts
sed -i 's/^PINZA_ABIERTA = .*/PINZA_ABIERTA = 105/' secuencia_gui.py
sed -i 's/^PINZA_CERRADA = .*/PINZA_CERRADA = 60/' secuencia_gui.py
sed -i 's/^Z_ABAJO = [0-9.]*/Z_ABAJO = 0.030/' secuencia_gui.py
grep -n "^PINZA_\|^Z_ABAJO\|^Z_BOTELLA" secuencia_gui.py
```

- `Z_ABAJO` está en metros. Si la pinza cierra antes de llegar a la tapa, sumarle lo que faltó (12 mm = 0.012).
- Si la pinza zumba fuerte al cerrar sobre la tapa, acercar `PINZA_CERRADA` a 90.
- Los ángulos de la pinza se pueden buscar con el deslizador de la ventana antes de fijarlos.

---

## 8. Solución de problemas

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| Un motor gira sin parar o no se mueve | Encoder o conector flojo, o sin 12 V | **Ctrl+C en la terminal 3**, revisar el conector y correr `PRUEBA` |
| `fin de la prueba (watchdog)` | El puente dejó de mandar comandos por 500 ms (PC lento o USB) | Si se repite, cerrar programas que no se usen |
| `El ESP32 se acaba de reiniciar` | Brownout: el servo hizo caer el voltaje | Revisar la fuente del servo o desconectarlo |
| `No llega la posición del ESP32` | Puerto ocupado o ESP32 sin arrancar | Cerrar miniterm o VS Code; si no arranca, desconectar U6 al conectar el USB (GPIO 12) |
| Los encoders dejan de contar al mover la pinza | Interferencia del servo activo | Verificar que `SOLTAR_PINZA = True` en `secuencia_gui.py` |
| La secuencia se detiene: "el robot real no llegó" | Una junta no alcanzó la pose en 60 s | Revisar esa junta con `PRUEBA`; volver a la pose cero y mandar `ZERO` |
| El robot real gira al revés que Gazebo | Signo de la junta | Cambiar su valor en `SIGNO` de `puente_serial.py` |

**Después de cualquier falla:** poner el robot en la pose cero y mandar `ZERO` antes de volver a correr, porque las cuentas pueden haber quedado desfasadas.

---

## 9. Apagado

1. Ctrl+C en las terminales 4, 3, 2 y 1, en ese orden.
2. Apagar la fuente de 12 V y la del servo.
3. Desconectar el USB.
