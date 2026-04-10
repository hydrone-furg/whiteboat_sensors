cmake_minimum_required(VERSION 3.8)
project(whiteboat_cam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Instala scripts Python como executáveis
install(PROGRAMS
  scripts/whiteboat_cam.py
  scripts/camera_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Instala arquivos de configuração de sensores
install(FILES my_sensors.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
