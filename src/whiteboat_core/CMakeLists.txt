cmake_minimum_required(VERSION 3.8)
project(whiteboat_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gera o serviço customizado RequestDataStream
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestDataStream.srv"
)

# Instala launch files e outros recursos
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/whiteboat_node.py
  scripts/square.py
  scripts/data_stream_server.py
  scripts/mavros_to_vrx_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
