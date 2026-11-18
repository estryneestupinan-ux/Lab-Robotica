# Celda Robotizada de Envasado de Aceite

Celda de cinco manipuladores que envasa aceite a **2 botellas por minuto**. Un nodo maestro en ROS 2 secuencia las operaciones y arbitra las zonas compartidas. La celda se validó en Gazebo y se replicó en físico con controladores ESP32.

<p align="center">
  <img src="media/images/render_celda.png" alt="Render de la celda completa" width="700">
</p>

---

## Descripción

La línea recibe el envase vacío, lo llena, lo transporta abierto, lo tapa y lo entrega. Cada paso lo hace un robot distinto, y todos se coordinan desde un único nodo maestro. Los robots no se comunican entre sí: reciben órdenes del maestro y le reportan su estado.

Documento: [Análisis del contexto (PDF)](docs/analisis_contexto.pdf)

| Robot | Función | Repositorio |
|-------|---------|-------------|
| R1 | ... | [Robot 1](../Robot_1/) |
| R2 | Alimentación de envases a la estación de llenado | [Robot 2](../Robot_2/) |
| R3 | Transporte del envase lleno: llenado → sellado | [Robot 3](../Robot_3/) |
| R4 | Tapado en la estación de sellado | [Robot 4](../Robot_4/) |
| R5 | ... | [Robot 5](../Robot_5/) |

---

## Plano de planta

<p align="center">
  <img src="media/images/plano_planta_celda.png" alt="Plano de planta de la celda" width="700">
</p>

### Recorrido del envase

| Tramo | Estación de origen | Estación de destino |
|-------|--------------------|---------------------|
| 1 | [estación de alimentación] | PLL (llenado) |
| 2 | PLL (llenado) | PS (sellado) |
| 3 | PS (sellado) | [estación de salida] |

PLL y PS son estaciones compartidas: en cada una intervienen dos robots en momentos distintos, y el maestro controla el acceso con candados (ver [Comunicación](#comunicación)).

### Secuencia de la celda

<p align="center">
  <img src="media/images/secuencia_celda.png" alt="Secuencia de operación de la celda" width="900">
</p>

---

## Funcionamiento de la planta

La planta envasa aceite de forma automática. Cada robot hace una tarea y le pasa el envase al siguiente:

1. **Robot 1:** ....
2. **Robot 2:** toma un envase vacío y lo coloca en la estación de llenado.
3. **Llenado:** el envase se llena con aceite en la estación de llenado.
4. **Robot 3:** toma el envase lleno, que todavía está abierto, y lo lleva a la estación de sellado. Lo mantiene vertical y se mueve sin aceleraciones bruscas para que no se derrame el aceite.
5. **Robot 4:** coloca la tapa al envase en la estación de sellado.
6. **Robot 5:** ....

Los robots trabajan al mismo tiempo: mientras el Robot 4 tapa un envase, el Robot 3 transporta el siguiente y el Robot 2 coloca otro. Así hay varios envases en proceso a la vez.

La planta produce **2 botellas por minuto**: sale una botella terminada cada 30 segundos. Un nodo maestro le avisa a cada robot cuándo le toca moverse, de modo que dos robots nunca estén en el mismo lugar al mismo tiempo.

- Cada estación debe terminar su tarea en 30 s o menos. La más lenta marca la velocidad de toda la planta.
- Si una estación se atrasa, el maestro no libera el siguiente envase.

---

## Máquina de estados

Máquina de estados de la celda que ejecuta el nodo maestro. Indica, según la acción del operario o del sistema, qué hace cada robot en cada estado.

<p align="center">
  <img src="media/images/maquina_estados.png" alt="Máquina de estados de la celda" width="900">
</p>

---

## Comunicación

El **nodo maestro** es el director de los robots: el programa que decide qué robot se mueve, cuándo y hacia dónde. Recibe los comandos del operario, envía las órdenes a los robots y recibe su estado. Todo se comunica por ROS 2 en la misma red.

Archivo: [nodo_maestro/maestro.py](nodo_maestro/maestro.py)

<p align="center">
  <img src="media/images/arquitectura_comunicacion.png" alt="Arquitectura de comunicación de la celda" width="850">
</p>

### Funciones del nodo maestro

1. Ejecuta la máquina de estados de la celda.
2. Administra los candados de PLL y PS.
3. Marca la cadencia: libera un envase cada 30 s.
4. Envía a cada robot su orden (estación destino, acción, trayectoria).
5. Supervisa timeouts, latido y fallas de cada robot.
6. Publica estado de la celda, estado por robot, zonas ocupadas, contador y cadencia real.

### Conexión con cada parte

| Parte | Conexión | Qué intercambia |
|-------|----------|-----------------|
| Interfaz del operario | ROS 2, misma red | Recibe los comandos del operario; publica estado, contador y alarmas |
| Gazebo | ROS 2, misma máquina | Envía órdenes a los controladores simulados; recibe estado y posición articular |
| ESP32 (robots reales) | Wi-Fi, a través del router | Envía órdenes y trayectorias; recibe estado, posición articular y fin de movimiento |

Gazebo y los ESP32 usan la misma interfaz de órdenes y estado hacia el maestro. Por eso el maestro es el mismo código en simulación y en la celda real; solo cambia el parámetro `modo` al lanzarlo (`simulacion` o `real`).

### Coordinación entre robots

Cada zona compartida tiene un **candado** que administra el maestro. El robot solo recibe la orden de entrar a una zona si el candado está libre, y el candado se libera cuando el robot reporta que ya salió.

| # | Regla | Motivo |
|---|-------|--------|
| 1 | Una zona compartida admite un robot a la vez | Evitar colisión directa |
| 2 | La orden de entrar a una zona se envía solo con el candado libre | El permiso es implícito: sin orden, el robot no entra |
| 3 | El candado se libera cuando el robot reporta estar **fuera** de la zona, no cuando termina de operar | El brazo puede seguir dentro al terminar de operar |
| 4 | R3 recibe PLL solo si PS está libre o se liberará a tiempo | Evitar que R3 espere con un envase lleno y abierto |
| 5 | Si un robot no reporta a tiempo, el candado no se libera y la celda pasa a `ERROR` | Un robot sin respuesta no se interpreta como zona libre |
| 6 | Si se pierde comunicación con un robot, el maestro deja de liberar envases | Una caída de Wi-Fi no debe generar un cruce |
| 7 | El paro de emergencia se publica a los 5 robots a la vez | Detención global |

### Red física

| Elemento | Medio | Función |
|----------|-------|---------|
| 5 ESP32 (uno por robot) | Wi-Fi | Reciben órdenes, ejecutan el control de bajo nivel y reportan estado |
| Router central | Red local | Enlaza los ESP32 con los equipos ROS 2 |
| Equipo con ROS 2 | Ethernet | Ejecuta el nodo maestro y la interfaz del operario |

---

## Interfaz del operario

Aplicación desde la que el operario arranca, supervisa y detiene la celda. Se comunica únicamente con el nodo maestro, nunca con un robot directamente, de modo que ninguna acción del operario puede saltarse los candados.

Código: [gui/](gui/)

### Abrir la interfaz

```bash
ros2 run celda_gui gui
```

<p align="center">
  <img src="media/images/gui_operario.png" alt="Interfaz gráfica del operario" width="800">
</p>

### Pantalla

| Panel | Qué muestra o permite |
|-------|-----------------------|
| Estado de la celda | Estado actual de la máquina de estados |
| Robots | Conexión, estado, posición articular y zona ocupada de cada uno de los 5 robots |
| Producción | Botellas producidas y cadencia real frente a la meta de 2 botellas/min |
| Controles | `INICIAR`, `PAUSAR` / `REANUDAR`, `HOME`, `RESET` |
| Paro de emergencia | Detención inmediata de toda la celda |
| Registro de eventos | Alarmas, fallas y cambios de estado con hora |

### Antes de iniciar

1. Celda energizada, router encendido y equipo ROS 2 en la red.
2. Zona de trabajo despejada: ninguna persona ni objeto dentro del alcance de los robots.
3. Envases vacíos disponibles en la entrada de la celda.
4. Interfaz abierta y los 5 robots en estado conectado en el panel **Robots**.

### Operación normal

| Paso | Acción del operario | Resultado esperado |
|------|---------------------|--------------------|
| 1 | Presionar `INICIAR` | La celda pasa a `HOMING`: cada robot va a su referencia |
| 2 | Esperar | Los 5 robots en HOME, estado `LISTO` y paso a `PRODUCCION` |
| 3 | Monitorear el panel **Producción** | El contador aumenta y la cadencia real se mantiene cerca de 2 botellas/min |
| 4 | `PAUSAR` si hay que intervenir | Cada robot termina su movimiento, llega a un punto seguro y la celda queda en `PAUSA` |
| 5 | `REANUDAR` | La celda vuelve a `PRODUCCION` |

### Estados y qué hacer en cada uno

| Estado | Significado | Acción del operario |
|--------|-------------|---------------------|
| `INICIO` | Celda sin movimiento | Presionar `INICIAR` |
| `HOMING` | Robots buscando su referencia | Esperar, no intervenir |
| `LISTO` | Los 5 robots en HOME | Esperar el paso a producción |
| `PRODUCCION` | Ciclo automático | Monitorear contador y cadencia |
| `PAUSA` | Robots detenidos en punto seguro | `REANUDAR`, o intervenir con la celda en este estado |
| `ERROR` | Falla detectada | Ver el registro de eventos, corregir la causa y presionar `RESET` |
| `PARO` | Emergencia activada | Revisar la causa y rearmar |

### Si aparece un error

1. Leer en el **Registro de eventos** qué robot falló y en qué paso.
2. No entrar a la zona de trabajo mientras haya movimiento.
3. Retirar o revisar el envase afectado y corregir la causa.
4. Presionar `RESET`: la celda repite el `HOMING` y queda lista para continuar.

### Paro de emergencia

Se usa cuando hay riesgo para una persona o para el equipo. Detiene los 5 robots de inmediato. Para volver a operar se revisa la causa, se rearma y se inicia de nuevo desde `INICIO`.

### Precauciones

- Intervenir en la celda solo en `PAUSA`, `ERROR` o `PARO`, nunca en `PRODUCCION`.
- Un envase con aceite derramado se limpia antes de reanudar.
- Si un robot aparece desconectado en el panel **Robots**, no iniciar la producción.

---

## Gazebo

Gazebo es el entorno donde se simula la celda completa: los 5 robots, las estaciones y los envases. Se usa de dos formas:

1. **Simulación:** el nodo maestro envía las órdenes y Gazebo mueve los robots simulados.
2. **Robots reales que mueven Gazebo:** los robots físicos se mueven y el modelo de Gazebo reproduce su posición.

**Requisitos:** Ubuntu [versión], ROS 2 [distribución], Gazebo [versión].

### 1. Simulación



https://github.com/user-attachments/assets/7a068337-a0e2-43c6-9f57-49952147cd16



**Qué se ve en el video**

1. La celda arranca en `HOMING` y los 5 robots van a su referencia.
2. Pasa a `PRODUCCION` y el maestro libera el primer envase.
3. R2 deposita el envase en PLL y sale; se simula el llenado.
4. R3 toma el envase, lo mantiene vertical y lo lleva a PS.
5. R4 coloca la tapa mientras R3 regresa para el siguiente ciclo.
6. Entra un envase nuevo cada 30 s, con varios robots operando a la vez.

**Cómo funciona**

- El maestro publica una orden por robot y cada controlador de articulación en Gazebo ejecuta la trayectoria recibida.
- La posición articular y el estado de cada robot simulado vuelven al maestro, que los usa para liberar los candados.
- Todos los nodos usan el reloj de simulación (`use_sim_time`).
- Como el maestro es el mismo código que en la celda real, aquí se prueban la secuencia, los candados y los tiempos de ciclo antes de mover el hardware.

**Cómo correrlo**

```bash
# Terminal 1: Gazebo con la celda
ros2 launch celda_gazebo celda.launch.py

# Terminal 2: nodo maestro en modo simulación
ros2 run celda_maestro maestro --ros-args -p modo:=simulacion -p use_sim_time:=true

# Terminal 3: interfaz del operario
ros2 run celda_gui gui
```

### 2. Robots reales que mueven Gazebo

[Arrastrar aquí el video de los robots reales moviendo Gazebo al editar este README en GitHub]

**Cómo funciona**

- Los ESP32 leen los encoders de cada articulación y publican la posición articular en ROS 2.
- Esa posición se aplica al modelo de Gazebo, de modo que el modelo se mueve igual que el robot físico.
- Permite comparar el movimiento real con el modelo (URDF, límites y cinemática) y supervisar la celda desde Gazebo.

**Cómo correrlo**

```bash
# Terminal 1: Gazebo con la celda
ros2 launch celda_gazebo celda_espejo.launch.py

# Terminal 2: nodo maestro en modo real
ros2 run celda_maestro maestro --ros-args -p modo:=real

# Terminal 3: interfaz del operario
ros2 run celda_gui gui
```

**Carpeta con los archivos de Gazebo: [Gazebo](Gazebo/)**

---

## Celda real

[Arrastrar aquí el video de la celda real al editar este README en GitHub]

<p align="center">
  <img src="media/images/foto_celda_real.jpg" alt="Celda real" width="700">
</p>

### Pruebas realizadas

| Prueba | Meta | Resultado |
|--------|------|-----------|
| Cadencia | 2 botellas/min | [completar] |
| Tiempo de ciclo por estación | ≤ 30 s | [completar] |
| Colisiones entre robots | Ninguna | [completar] |
| Paro de emergencia | Todos los robots se detienen | [completar] |

---

**Laboratorio de Robótica | Universidad Militar Nueva Granada.**
