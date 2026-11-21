# Verificar que el SCARA va a donde le pedimos

Archivos:
- `scara_verificacion.ino`: programa del ESP32. Hace el home, controla cada eje con su encoder y calcula la cinemática inversa.
- `visor_scara.html`: visor en el navegador. Muestra el robot desde arriba y lo mueves con clics. Ábrelo con **Edge o Chrome**.

Sin el robot conectado, el visor funciona en **modo simulación**: puedes hacer clic y ver cómo se movería.

---

## 0. Antes de empezar
- La fuente debe estar en **24 V** y los finales de carrera conectados: C a 3.3 V y NO a GPIO21 / 22 / 15.
- Robot firme en la mesa y nada en su camino. Ten a mano el botón **BOOT** del ESP32 o la tecla **Esc** del visor: los dos lo **paran**.
- Sube `scara_verificacion.ino` (placa DOIT ESP32 DEVKIT V1).
- **Cierra el Monitor serie de Arduino** antes de abrir el visor: el puerto solo lo puede usar un programa a la vez.
- Abre `visor_scara.html`, pulsa **Conectar ESP32** y elige el puerto COM.

## 1. Home y sentido de giro
1. Pulsa **Home**. Se hace en este orden: Z, codo y hombro. Cada eje va despacio hasta su final de carrera.
   - **Si un eje se aleja de su switch** en lugar de ir hacia él: pulsa PARAR, escribe `set 1 dirhome -1` (o `2`, o `z`) y repite el home.
   - El sentido del encoder lo detecta el programa solo.
2. Comprueba el sentido de cada articulación con `jog`.
   - **Hombro:** escribe `jog 1 10` y míralo desde arriba.
     - Si gira en **sentido antihorario**, está bien.
     - Si gira en sentido horario: escribe `set 1 alejar -1` y `home 1`.
   - **Codo:** escribe `jog 2 10` y mira el brazo 2 respecto al brazo 1. Debe girar en sentido antihorario visto desde arriba; si no, escribe `set 2 alejar -1` y `home 2`.
   - **Eje Z:** escribe `jog z -5`. Debe **bajar**; si sube, escribe `set z alejar 1` y `home z`.

## 2. Calibrar (cuentas por grado y por mm)
El programa trae valores aproximados. Hay que medirlos una vez.

**Hombro:**
1. Después del home, escribe `marca` y dibuja en la mesa una línea a lo largo del brazo 1.
2. Escribe `jog 1 90`.
3. Mide con un transportador o una escuadra cuántos grados giró **de verdad**, por ejemplo 84°.
4. Escribe `k 1 84`. El programa corrige el valor.
5. Comprueba con `jog 1 -90`: debe volver a la línea.

**Codo:** lo mismo con `marca`, `jog 2 90`, medir el ángulo entre las dos posiciones del brazo 2, y `k 2 <grados>`.

**Eje Z:**
1. Escribe `marca` y mide la altura de la punta de la cremallera.
2. Escribe `jog z -10`.
3. Mide cuánto bajó **de verdad** y escribe `k z <mm>`.

**Dónde está el home de cada eje** (`set E home V`):
- **Hombro:** déjalo en `0`. Así el eje **x** es la dirección del brazo 1 cuando su final de carrera está activo.
- **Codo:** en home, mide el ángulo entre el brazo 2 y la prolongación del brazo 1.
  - 0° = brazos en línea recta.
  - Positivo si el brazo 2 queda girado en sentido antihorario visto desde arriba.
  - Ejemplo: `set 2 home -120`.
- **Z:** con el switch activo, mide la altura de la punta de la cremallera sobre la mesa, en mm. Ejemplo: `set z home 108`.

**Cuánto se puede mover cada eje desde el home** (`set E rango V`): por ejemplo `set 1 rango 170`.

Al terminar, escribe `cfg` y copia las 3 líneas que aparecen en `Config CFG[3]` del código. Súbelo otra vez: así la calibración queda guardada.

## 3. Verificar la cinemática
Pega una hoja en la mesa:
- marca el **origen O**: el punto justo debajo del eje del hombro (el centro de la base);
- dibuja el **eje x**: la dirección del brazo 1 en home.

El eje **y** queda a 90° a la izquierda de x, visto desde arriba. Si le pones un plumón a la pinza, puedes marcar los puntos directamente.

**Prueba A: puntos sueltos**
1. Escribe `xyz 200 100 100` o haz clic en el visor.
2. Cuando diga "llego", mide con regla dónde quedó la punta, en x y en y desde O.
3. En el visor pulsa **Registrar punto actual** y escribe la x y la y medidas. La tabla calcula el error.
4. Repite con unos 8 puntos repartidos por la zona azul: cerca, lejos, izquierda y derecha.

**Prueba B: cuadrado** (no necesita medir desde el origen)
1. Pulsa **Cuadrado** o escribe `cuadrado 150 80 80`. El robot hace una pausa de 3 s en cada esquina para que la marques.
2. Mide los 4 lados (deben dar 80 mm) y las 2 diagonales (deben dar 113.1 mm).
3. Si los lados salen más largos o más cortos de forma pareja, la calibración de cuentas por grado está mal.
4. Si el cuadrado sale torcido o como un rombo, revisa el home del codo (`set 2 home`).

**Cómo leer la tabla:**
- **error enc**: diferencia entre lo pedido y lo que dicen los encoders. Mide el **control**: si es grande, hay que ajustar `tol`, `kp` o `minpwm`.
- **error real**: diferencia entre lo pedido y lo medido con la regla. Mide **todo junto**: control, calibración, medidas L1/L2 y juego de los engranajes.

**Descargar CSV** guarda la tabla para tu informe.

## 4. Si algo falla
| Mensaje | Qué hacer |
|---|---|
| `no encontro el final de carrera` | el eje iba al revés: `set E dirhome ...` |
| `el encoder no cuenta` | revisa A/B, 5 V y GND del encoder de ese motor |
| `no se mueve (trabado...)` | el motor no tiene fuerza o está trabado. Revisa la fuente; sube `minpwm` si el motor zumba sin moverse |
| oscila alrededor del punto | baja `kp` o sube `tol` (ej. `set 1 tol 1.5`) |
| se queda corto del punto | baja `tol` o sube `minpwm` |
| `fuera de alcance` / `fuera de los limites` | el punto no se puede alcanzar con L1 = 120, L2 = 188.4 y los límites actuales |

Después de `reset` hay que volver a hacer **Home** si el eje perdió la posición.
