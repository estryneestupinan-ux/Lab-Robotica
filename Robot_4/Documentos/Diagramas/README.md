# Diagramas del SCARA R4

GitHub dibuja los diagramas de este archivo directamente (Mermaid). La imagen `arquitectura.png` es la misma Figura 6 de la cartilla.

1. [Arquitectura del sistema](#arquitectura-del-sistema)
2. [Diagrama de secuencia del proceso](#diagrama-de-secuencia-del-proceso)
3. [Diagrama de flujo de la secuencia (`secuencia_gui.py`)](#diagrama-de-flujo-de-la-secuencia)
4. [Diagrama de flujo del puente (`puente_serial.py`)](#diagrama-de-flujo-del-puente-serial)
5. [Diagrama de flujo del firmware del ESP32](#diagrama-de-flujo-del-firmware-del-esp32)

---

## Arquitectura del sistema

![Arquitectura](arquitectura.png)

```mermaid
flowchart LR
    subgraph PC["Computador con ROS 2"]
        SEQ["Planeador<br/>secuencia_gui.py"]
        GZ["Gazebo + ros2_control"]
        HOLD["joint3_position_hold.py<br/>lazo P del eje Z"]
        BR["Nodo puente<br/>puente_serial.py"]
        CSV[("CSV Gazebo vs. real")]
    end
    subgraph R4["Robot R4"]
        ESP["ESP32<br/>lazo de posición cada 20 ms"]
        DRV["2 × TB6612FNG"]
        MOT["Motores JGB37 ×2 y N20<br/>con encoder Hall"]
        SRV["Servo de la pinza<br/>GPIO 23"]
    end
    SEQ -- "/joint1, /joint2 commands" --> GZ
    SEQ -- "/joint3_target" --> HOLD
    HOLD -- "/joint3_velocity_controller" --> GZ
    GZ -- "/joint_states" --> BR
    SEQ -- "/pinza" --> BR
    BR -- "/real/joint_states" --> SEQ
    BR --> CSV
    BR -- "SET:c1,c2,c3 · SERVO:θ" --> ESP
    ESP -- "POS:t,c1,c2,c3,..." --> BR
    ESP -- "PWM y sentido" --> DRV
    DRV -- "12 V" --> MOT
    MOT -- "pulsos A y B" --> ESP
    ESP -- "pulso 50 Hz" --> SRV
```

---

## Diagrama de secuencia del proceso

Un ciclo de colocación de tapa, con el robot real conectado.

```mermaid
sequenceDiagram
    actor Op as Operador
    participant SEQ as secuencia_gui.py
    participant GZ as Gazebo
    participant BR as puente_serial.py
    participant ESP as ESP32
    participant HW as Motores, encoders y pinza

    Op->>SEQ: Iniciar Secuencia
    SEQ->>BR: /pinza = 105° (abrir)
    BR->>ESP: SERVO:105
    ESP->>HW: pulsos al servo
    SEQ->>BR: /pinza = −1 (soltar)
    BR->>ESP: SERVO:OFF
    loop Cada tramo (Home, dispensador, bajar, subir, envase, bajar, subir, Home)
        loop Cada 33 ms durante T segundos
            SEQ->>GZ: consigna smoothstep de q1, q2, d3
        end
        loop Cada 20 ms
            GZ-->>BR: /joint_states
            BR->>ESP: SET:c_Z,c_codo,c_base (velocidad limitada)
            ESP->>HW: PWM de cada motor
            HW-->>ESP: cuentas de encoder
            ESP-->>BR: POS:t,c1,c2,c3,pwm
            BR-->>SEQ: /real/joint_states
        end
        SEQ->>SEQ: esperar a que el robot real llegue (tolerancia, máx. 60 s)
        opt Tramo que termina abajo
            SEQ->>BR: /pinza = 60° (cerrar) o 105° (abrir)
            BR->>ESP: SERVO:θ
            SEQ->>BR: /pinza = −1
            BR->>ESP: SERVO:OFF
        end
    end
    SEQ-->>Op: Secuencia completada
```

---

## Diagrama de flujo de la secuencia

```mermaid
flowchart TD
    A([Iniciar Secuencia]) --> B[Tomar la pose actual de Gazebo]
    B --> C[Abrir la pinza 105° y soltar el servo]
    C --> D{¿Quedan tramos?}
    D -- No --> Z([Secuencia completada])
    D -- Sí --> E["Elegir q1 objetivo ±2π dentro de [−3.0, 3.2] rad"]
    E --> F["T = máx(T_nom, 1.5·|Δq|/v) por junta"]
    F --> G["Publicar q(t) smoothstep a 30 Hz durante T"]
    G --> H{¿Hay robot real?<br/>datos en /real/joint_states}
    H -- No --> K
    H -- Sí --> I{¿Llegó a la pose<br/>dentro de la tolerancia?}
    I -- Sí --> K{¿El tramo mueve la pinza?}
    I -- No --> J{¿Pasaron 60 s?}
    J -- No --> I
    J -- Sí --> X([Detener: el robot real no llegó])
    K -- No --> P[Pausa 0.5 s]
    K -- Sí --> L[Esperar 1 s para que Z asiente]
    L --> M[Cerrar 60° o abrir 105°]
    M --> N["Esperar recorrido/90 °/s + 0.8 s"]
    N --> O[Soltar el servo: SERVO:OFF]
    O --> P
    P --> D
```

---

## Diagrama de flujo del puente serial

```mermaid
flowchart TD
    A([Inicio]) --> B[Abrir /dev/esp32 con DTR y RTS en alto<br/>para no reiniciar el ESP32]
    B --> C[TEL:1 y esperar la primera línea POS]
    C --> D[Consigna inicial = posición medida del robot]
    D --> E[Crear el CSV robot_real_fecha.csv]
    E --> F{{Cada 20 ms}}
    F --> G{¿El ESP32 se reinició?}
    G -- Sí --> H[Dejar de enviar consignas y avisar]
    G -- No --> I[Objetivo = /joint_states de Gazebo,<br/>recortado a LIMITES]
    I --> J["Avanzar la consigna hacia el objetivo<br/>como máximo VEL_MAX·Δt por junta"]
    J --> K["Cuentas = round(SIGNO·k·q)"]
    K --> L[Enviar SET:c_Z,c_codo,c_base]
    L --> M{¿Llegó /pinza?}
    M -- "θ ≥ 0" --> N[Enviar SERVO:θ]
    M -- "θ < 0" --> O[Enviar SERVO:OFF]
    M -- No --> F
    N --> F
    O --> F
    P{{Hilo de lectura}} --> Q[Leer línea del ESP32]
    Q --> R{¿POS?}
    R -- Sí --> S[Convertir a unidades de junta,<br/>publicar /real/joint_states<br/>y escribir una fila del CSV]
    R -- No --> T[Mostrar MSG o ERR]
    S --> Q
    T --> Q
```

---

## Diagrama de flujo del firmware del ESP32

```mermaid
flowchart TD
    A([Encendido]) --> B[setup: pines IN1/IN2, PWM 20 kHz,<br/>interrupciones de encoder, servo sin pulsos]
    B --> C[Timer de hardware: alarma cada 20 ms]
    C --> D{{loop}}
    D --> E{¿El timer levantó la bandera?}
    E -- No --> S[leerSerial]
    E -- Sí --> F[Contar ticks perdidos]
    F --> G[pasoControl]
    G --> H[pasoServo: rampa del pulso a 90 °/s]
    H --> S
    S --> T{¿Línea completa?}
    T -- No --> D
    T -- Sí --> U[procesarLinea: SET, STOP, ZERO, PRUEBA,<br/>SERVO, MODO, WD, TEL, INFO]
    U --> D

    G --> G1[Leer las tres cuentas]
    G1 --> G2{¿Más de 500 ms sin SET?}
    G2 -- Sí --> G3[Apagar motores: modo ESPERA]
    G2 -- No --> G4["Para cada motor: e = (consigna − cuentas)/k"]
    G4 --> G5{Modo de control}
    G5 -- P --> G6["|e| ≤ tol → 0; si no,<br/>sign(e)·min(u_max, 35 + Kp|e|)"]
    G5 -- LGR --> G7["u = u_prev + K·(e − β·e_prev),<br/>saturado y anti-windup"]
    G6 --> G8[Escribir PWM y sentido al TB6612FNG]
    G7 --> G8
    G8 --> G9[Enviar POS:t,c1,c2,c3,pwm1,pwm2,pwm3]

    I1([Flanco de subida del canal A]) --> I2{¿A sigue en alto?}
    I2 -- No --> I3[Descartar: pico de ruido]
    I2 -- Sí --> I4{¿Canal B en alto?}
    I4 -- Sí --> I5[cuentas − 1]
    I4 -- No --> I6[cuentas + 1]
```
