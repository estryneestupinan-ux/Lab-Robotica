# Planos

Esta carpeta guarda los planos mecánicos y eléctricos del SCARA. Mientras se agregan, estas son las medidas del prototipo construido.

## Dimensiones principales

| Elemento | Valor |
|---|---|
| Brazo 1, eje del hombro a eje del codo (L1) | 120 mm |
| Brazo 2, eje del codo a eje de la cremallera (L2) | 188.6 mm |
| Carrera de la cremallera | 85 mm |
| Altura de la punta sobre la mesa | 162 mm (abajo, home) a 247 mm (arriba) |
| Alcance radial | 68.6 a 308.6 mm |
| Apertura de la pinza (separación entre dedos) | 11.4 mm cerrada · 47.3 mm abierta |

## Recorridos de las juntas

| Junta | Recorrido | Referencia (home) |
|---|---|---|
| Hombro | 120° | Final de carrera; −92° desde el eje x |
| Codo | 180° | Final de carrera; +88° desde el brazo estirado |
| Eje Z | 85 mm | Final de carrera abajo; punta a 162 mm |

## Conexiones eléctricas

| Elemento | Pines del ESP32 |
|---|---|
| Motor del hombro (driver B2) | RPWM 25 · LPWM 26 · encoder 16/17 · final de carrera 21 |
| Motor del codo (driver B1) | RPWM 27 · LPWM 14 · encoder 18/19 · final de carrera 22 |
| Motor del eje Z (driver B3) | RPWM 32 · LPWM 33 · encoder 4/23 · final de carrera 15 |
| Habilitación de los drivers | GPIO 5 |
| Servo de la pinza | GPIO 13 |
| Parada | Botón BOOT (GPIO 0) |

Los finales de carrera se conectan con C a 3.3 V y NO al pin del ESP32 (presionado = 3.3 V).

## Archivos pendientes

- Plano de ensamble mecánico (CAD exportado a PDF).
- Plano de piezas impresas en 3D.
- Esquemático eléctrico de la tarjeta (ESP32, 3 drivers BTS7960, fuente de 24 V).
