# whiteboat_cam

Este pacote é responsável pela simulação e processamento de câmera para o Whiteboat em ROS 2.

## Pré-requisitos

Certifique-se de ter o ambiente ROS 2 Humble configurado.

## Passos para Compilação:

```bash
cd ~/ros2_ws
colcon build --packages-select whiteboat_cam
source install/setup.bash
```

## Execução

### Para executar o nó de simulação de câmera:

```bash
ros2 run whiteboat_cam camera_sim_node.py
```

### Para executar via launch file:

```bash
ros2 launch whiteboat_cam whiteboat_cam.launch.py
```
