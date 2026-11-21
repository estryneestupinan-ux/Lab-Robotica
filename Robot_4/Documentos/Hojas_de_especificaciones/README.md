# Hojas de especificaciones de los dispositivos

Hojas de datos oficiales de los componentes del R4 y los valores que se usaron en el diseño. Para los motorreductores genéricos (JGB37-520, N20) no hay hoja oficial de un fabricante único: se listan los datos del vendedor y lo que se **midió** en el robot.

| Dispositivo | Uso en el R4 | Datos usados en el diseño | Hoja de datos |
|---|---|---|---|
| **ESP32** (Espressif) | Control de bajo nivel | Doble núcleo hasta 240 MHz; timers de hardware de 64 bits; PWM LEDC de hasta 16 canales; interrupciones en cualquier GPIO; los GPIO 34–39 son solo entrada y el GPIO 12 es pin de arranque (voltaje de la flash) | [ESP32 Series Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf) |
| **TB6612FNG** (Toshiba) ×2 | Puentes H de los tres motores | VM hasta 15 V; 1.2 A continuos y 3.2 A pico por canal; lógica de 2.7 a 5.5 V; PWM hasta 100 kHz | [Toshiba TB6612FNG](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660) |
| **L7805** (ST) | Regulador de 5 V (ESP32, lógica y encoders) | Salida de 5 V; hasta 1.5 A con disipación adecuada; entrada de hasta 35 V | [ST L78 series](https://www.st.com/en/power-management/l78.html) |
| **JGB37-520**, 12 V, 12 rpm ×2 | Base y codo | Reductor 810:1; encoder Hall de 11 pulsos/vuelta en el eje del motor; ≈ 1.47 N·m nominal (vendedor). Medido: 8910 cuentas por vuelta de salida, 90° exactos con 2227 cuentas | — |
| **N20**, 12 V, 30 rpm | Eje Z | Encoder de 7 pulsos/vuelta; reducción ≈ 1000:1 (deducida de la prueba de motores). Medido: 7000 cuentas por vuelta, ≈ 4 mm/s máximo con el husillo | — |
| **Husillo T8×8** | Eje Z | Paso 2 mm, 4 entradas: avance de 8 mm por vuelta (confirmado: 10 mm pedidos = 10 mm medidos) | — |
| **Correa dentada 20:60** | Transmisión del codo | Reducción 3:1: triplica torque y resolución, divide la velocidad entre tres | — |
| **Microservo de 50 Hz** | Pinza | Pulso de 500 a 2500 µs para 0 a 180°; calibrado a 105° (abierta) y 60° (cerrada); sostiene la tapa sin pulsos | — |

## Limitaciones que impusieron las hojas de datos

- **GPIO 34 y 35** solo pueden ser entradas: el esquemático inicial los usaba como PWM del driver U7, y el cableado final pasó el PWM a los GPIO 16 y 17 ([Planos/Electricos](../../Planos/Electricos/README.md)).
- **GPIO 12** (MTDI) fija el voltaje de la flash al arrancar. Tener el canal A del encoder de la base ahí puede impedir el arranque (ver [firmware_esp32](../../firmware_esp32/README.md)).
- **TB6612FNG**: 1.2 A continuos por canal sobran para los motorreductores; el límite real fue la fuente de 12 V, que debe sostener los tres motores andando a la vez.
