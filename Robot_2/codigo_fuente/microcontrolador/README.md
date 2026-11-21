# Firmware del ESP32

## `scara_verificacion/`

| Archivo | Uso |
|---|---|
| `scara_verificacion.ino` | Programa del ESP32: home, lazo de posición de cada eje con su encoder, cinemática inversa, pinza y protocolo serie |
| `GUIA_VERIFICACION.md` | Home, sentido de giro, calibración de cuentas por grado y por mm, y pruebas de la cinemática |
| `visor_scara.html` | Visor en el navegador (Edge o Chrome): muestra el robot desde arriba y lo mueve con clics |
| `puente_esp32.py` | Puente para el modelo anterior (`Scara_Final_description`). Usa el comando `gz` y **no sirve** con `Robrot_description`; para el modelo actual usa `espejo_gazebo_real.py` o `puente_robot_real.py` |

## Cargar el firmware

1. Abre `scara_verificacion.ino` en Arduino IDE.
2. Placa: **DOIT ESP32 DEVKIT V1**.
3. Sube el programa y cierra el monitor serie antes de usar ROS o el visor.

## Qué hace

- Al arrancar deja la pinza cerrada y espera. Con `home` busca los finales de carrera en orden: **Z, codo y hombro**.
- Cada 5 ms controla los tres ejes con un lazo proporcional que compensa la zona muerta del motor, con histéresis, rampa de aceleración y detección de atasco.
- Con `rep 1` envía su estado cada 100 ms:

  ```text
  @,estado,hombro_deg,codo_deg,z_mm,t1,t2,tz,x_mm,y_mm,homed,fc
  ```

- Recibe objetivos de posición con `qq <hombro°> <codo°> <z mm>` (solo después del home) y mueve la pinza con `pinza <25…145>`.
- Se detiene con `s` o con el botón **BOOT**.

Los parámetros de cada eje están en el arreglo `CFG[3]`; después de calibrar con la guía, copia la salida del comando `cfg` en ese arreglo y vuelve a subir el programa.

El protocolo completo y la conversión entre los ángulos del firmware y los del modelo de ROS están en la [arquitectura](../../documentacion/arquitectura/README.md) y en las [memorias de cálculo](../../documentacion/memorias_calculo/README.md).
