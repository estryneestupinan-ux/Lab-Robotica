# Celda Robotizada de Envasado de Aceite
---

## Descripción
Celda de cinco manipuladores que envasa aceite a **2 botellas por minuto**. Un nodo maestro en ROS 2 secuencia las operaciones y arbitra las zonas compartidas. La celda se validó en Gazebo y se replicó en físico con controladores ESP32. La línea recibe el envase vacío, lo llena, lo transporta abierto, lo tapa y lo entrega. Cada paso lo hace un robot distinto, y todos se coordinan desde un único nodo maestro. Los robots no se comunican entre sí: reciben órdenes del maestro y le reportan su estado.

Documento: [Analisis_de_Contexto_General_Planta_Aceite.docx.pdf](https://github.com/user-attachments/files/35245023/Analisis_de_Contexto_General_Planta_Aceite.docx.pdf)


| Robot | Función | Repositorio |
|-------|---------|-------------|
| R1 | ... | [Robot 1](../Robot_1/) |
| R2 | Alimentación de envases a la estación de llenado | [Robot 2](../Robot_2/) |
| R3 | Transporte del envase lleno: llenado → sellado | [Robot 3](../Robot_3/) |
| R4 | Tapado en la estación de sellado | [Robot 4](../Robot_4/) |
| R5 | ... | [Robot 5](../Robot_5/) |

---

## Funcionamiento de la planta

La planta envasa aceite de forma automática. Cada robot hace una tarea y le pasa el envase al siguiente:

1. **Robot 1:** ....
2. **Robot 2:** toma un envase vacío y lo coloca en la estación de llenado.
3. **Llenado:** el envase se llena con aceite en la estación de llenado.
4. **Robot 3:** toma el envase lleno, que todavía está abierto, y lo lleva a la estación de sellado. Lo mantiene vertical y se mueve sin aceleraciones bruscas para que no se derrame el aceite.
5. **Robot 4:** coloca la tapa al envase en la estación de sellado.
6. **Robot 5:** ....


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


### Pruebas realizadas

| Prueba | Meta | Resultado |
|--------|------|-----------|
| Cadencia | 2 botellas/min | [completar] |
| Tiempo de ciclo por estación | ≤ 30 s | [completar] |
| Colisiones entre robots | Ninguna | [completar] |
| Paro de emergencia | Todos los robots se detienen | [completar] |

---

**Laboratorio de Robótica | Universidad Militar Nueva Granada.**
