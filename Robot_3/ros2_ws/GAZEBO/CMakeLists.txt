cmake_minimum_required(VERSION 3.8)
project(urdfscarafinal)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/ejecutar_tareas.py
  scripts/tareas_tabla.py
  scripts/planificador_trayectorias.py
  scripts/graficar_resultados.py
  scripts/verificar_choques.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
