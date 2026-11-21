# Diagramas de secuencia

## 1. Arranque del sistema completo (Gazebo + robot físico)

```mermaid
sequenceDiagram
    actor U as Usuario
    participant L as gazebo.launch.py
    participant G as Gazebo + gz_ros2_control
    participant E as espejo_gazebo_real.py
    participant M as ESP32
    participant C as control_cinematico.py

    U->>L: ros2 launch Robrot_description gazebo.launch.py
    L->>G: abre Gazebo y crea el robot
    L->>G: carga joint_state_broadcaster
    L->>G: carga velocity_controller
    G-->>E: /joint_states (pose simulada)

    U->>E: python3 espejo_gazebo_real.py -p puerto:=/dev/ttyUSB0
    E->>M: abre el puerto USB
    Note over M: el ESP32 se reinicia al abrir el puerto
    E->>E: espera 3 s
    E->>M: rep 1, reset, home
    M->>M: home de Z, luego codo, luego hombro
    M-->>E: @,...,homed=111
    Note over E: "Siguiendo a Gazebo."

    U->>C: ros2 run ... control_cinematico.py -p trayectoria:=T4
    loop cada 20 ms
        C->>G: /velocity_controller/commands
        G-->>C: /joint_states
    end
    loop cada 50 ms
        G-->>E: /joint_states
        E->>M: qq hombro codo z
        M-->>E: @ estado (cada 100 ms)
    end
```

## 2. Ciclo de una trayectoria con evento de pinza

Así recorre `control_cinematico.py` cada trayectoria. Los eventos de la pinza se definen con el parámetro `pinza_eventos` (por defecto `"T4:2,T7:3"`).

```mermaid
stateDiagram-v2
    [*] --> esperando
    esperando --> al_inicio: toma la siguiente trayectoria
    al_inicio --> siguiendo: error < 0.005 rad o m (o 15 s)
    note right of al_inicio: solo término proporcional
    siguiendo --> al_final: termina el tiempo de la referencia
    note right of siguiendo: velocidad de referencia + Kp·error, saturada
    al_final --> pinza_abre: la trayectoria tiene evento
    al_final --> esperando: sin evento y quedan trayectorias
    al_final --> [*]: sin evento y no quedan trayectorias
    pinza_abre --> pinza_espera: servo abierto
    pinza_espera --> pinza_cierra: pasan 2 s (T4) o 3 s (T7)
    pinza_cierra --> esperando: servo cerrado y quedan trayectorias
    pinza_cierra --> [*]: no quedan trayectorias
```

| Al terminar | Acción de la pinza | Tiempo abierta |
|---|---|---|
| T4 | abre, espera y cierra | 2 s |
| T7 | abre, espera y cierra | 3 s |
| T1, T8, T19 | permanece cerrada | — |

## 3. Enlace Gazebo → robot físico, cada 50 ms

```mermaid
sequenceDiagram
    participant G as Gazebo
    participant E as espejo_gazebo_real.py
    participant M as ESP32

    G-->>E: /joint_states: q1, q2, d3, servo
    E->>E: recorta q1, q2 y d3 a los límites del robot real
    E->>E: avanza el objetivo como máximo 0.20 rad/s y 0.15 m/s
    E->>E: convierte al firmware: θ1 = q1 − 138.11°, θ2 = q2 − 46.47°, z = 247 − 1000·d3
    E->>M: qq θ1 θ2 z
    E->>M: pinza 25…145 (si cambió 1° o más)
    M->>M: lazo de posición por eje cada 5 ms
    M-->>E: @,estado,θ1,θ2,z,...,homed,fc
    E-->>E: publica /robot_real/joint_states
    alt el ESP32 reporta FALLA
        E->>E: deja de enviar y avisa
    end
```

## 4. Detener el robot

| Situación | Qué pasa |
|---|---|
| `Ctrl + C` en el control | Envía velocidad cero a todas las juntas |
| `Ctrl + C` en el espejo | Envía `s` al ESP32 y cierra el puerto |
| Botón BOOT del ESP32 | Detiene todos los ejes de inmediato |
| Eje trabado | El firmware sube el PWM de a poco; si aun así no se mueve, entra en FALLA y para |
