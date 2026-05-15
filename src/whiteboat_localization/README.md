# whiteboat_localization

Este pacote gerencia a localização do Whiteboat em ROS 2.

## Pré-requisitos

Certifique-se de ter o ambiente ROS 2 Humble configurado.

## Passos para Compilação:

```bash
cd ~/ros2_ws
colcon build --packages-select whiteboat_localization
source install/setup.bash
```

## Execução

### Para executar o nó de localização:

```bash
ros2 run whiteboat_localization localization_node.py
```

### Para executar via launch file:

```bash
ros2 launch whiteboat_localization localization.launch.py
```