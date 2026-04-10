cmake_minimum_required(VERSION 3.8)
project(whiteboat_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gera o serviço customizado RequestDataStream
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestDataStream.srv"
)

# Instala o módulo Python whiteboat_tools (em src/)
ament_python_install_package(whiteboat_tools
  PACKAGE_DIR src/whiteboat_tools
)

# Instala scripts executáveis
install(PROGRAMS
  scripts/whiteboat_node.py
  scripts/square.py
  scripts/data_stream_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instala launch files e outros recursos
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
