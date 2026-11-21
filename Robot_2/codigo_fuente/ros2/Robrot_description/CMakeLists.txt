cmake_minimum_required(VERSION 3.10)
project(Robrot_description)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config trayectorias
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/control_cinematico.py
  scripts/graficar.py
  scripts/puente_robot_real.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
