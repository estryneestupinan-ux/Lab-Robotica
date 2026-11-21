# Documentación del SCARA R4

Índice de la documentación de soporte del Robot 4. Lo más completo y actualizado es la **cartilla técnica de la entrega final**, que integra todo el semestre.

## Entrega final

| Documento | Contenido |
|---|---|
| [Cartilla_SCARA_R4_Entrega_Final.pdf](Cartilla_SCARA_R4_Entrega_Final.pdf) | Cartilla técnica: problema, requerimientos, diseño mecánico y pinza, cinemática, simulación, hardware, control, integración Gazebo–robot real, planeación, validación, resultados, limitaciones y trabajo futuro |
| [Video del funcionamiento](https://drive.google.com/file/d/1xr8ooY4bUz9gxodWNE3gWuyT3NXSXv7d/view?usp=sharing) | La tarea completa en el robot real, con la explicación de cada etapa |
| [Instructivo_de_usuario.md](Instructivo_de_usuario.md) | Conexiones, verificación del ESP32, arranque, ejecución de la tarea, ajustes y solución de problemas |
| [Memorias_de_calculo.md](Memorias_de_calculo.md) | Constantes de conversión, velocidades, cinemática y poses, duración de la secuencia, controlador LGR, ganancias P y servo |
| [Diagramas/](Diagramas/README.md) | Arquitectura del sistema, diagrama de secuencia del proceso y diagramas de flujo de la secuencia, el puente y el firmware |
| [Hojas_de_especificaciones/](Hojas_de_especificaciones/README.md) | Hojas de datos de ESP32, TB6612FNG y L7805, y datos de motores, husillo, correa y servo |
| [datos/](datos/README.md) | Registro de la recta cartesiana de validación (`trayectoria_datos.csv`) |
| [Figuras/](Figuras) | Imágenes usadas en los README y en la cartilla |

Los planos mecánicos y eléctricos están en [../Planos](../Planos/README.md).

## Entregas parciales del semestre

En orden cronológico:

| Documento | Contenido |
|---|---|
| [Analisis Contexto - Robot tapas en envases de aceite caliente 1.pdf](Analisis%20Contexto%20-%20Robot%20tapas%20en%20envases%20de%20aceite%20caliente%201.pdf) | Análisis del contexto: la línea de envasado, la tarea del R4 y sus restricciones |
| [analisis_cinematico_R4-2.pdf](analisis_cinematico_R4-2.pdf) | Comparación de arquitecturas antropomórfica (RRR) y SCARA (RRP) por cinemática directa; longitudes y rangos articulares |
| [Informe_Tecnico_Lab_Robotica.pdf](Informe_Tecnico_Lab_Robotica.pdf) | Caracterización del motor DC, identificación de la planta y diseño del controlador discreto por LGR en el ESP32 (agosto de 2026) |
| [robotica.pdf](robotica.pdf) | Modelado y verificación cinemática del R4: Denavit–Hartenberg, directa e inversa |
| [Informe_Tecnico_SCARA_R4_Trayectoria(1)(1).docx.pdf](Informe_Tecnico_SCARA_R4_Trayectoria%281%29%281%29.docx.pdf) | Trayectoria cartesiana en línea recta y análisis de torques en ROS 2 y Gazebo (versión corregida; [Informe_Tecnico_SCARA_R4_Trayectoria.pdf](Informe_Tecnico_SCARA_R4_Trayectoria.pdf) es la versión anterior) |
| [PlaneacionDeTrayectoria.pdf](PlaneacionDeTrayectoria.pdf) | Planeación de trayectoria en el espacio de articulaciones para la tarea (septiembre de 2026) |
| [SCARA_Robot4_Presentacion.pptx.pdf](SCARA_Robot4_Presentacion.pptx.pdf) | Presentación del proyecto, versión anterior a la pinza y al robot real |
| [arquitectura.jpeg](arquitectura.jpeg) | Diagrama de arquitectura de la primera etapa (reemplazado por [Diagramas/arquitectura.png](Diagramas/arquitectura.png)) |
