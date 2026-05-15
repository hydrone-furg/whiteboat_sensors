# whiteboat_navigation

Este pacote é responsável pela navegação autônoma do Whiteboat em ROS 2, incluindo a execução de trajetórias em quadrado.

## Pré-requisitos

Certifique-se de ter o ambiente ROS 2 Humble configurado.

## Passos para Compilação:

```bash
cd ~/ros2_ws
colcon build --packages-select whiteboat_navigation
source install/setup.bash
```

## Execução

### Para executar o nó de navegação em quadrado (square node):

```bash
ros2 run whiteboat_navigation square.py
```

### Para executar via launch file:

```bash
ros2 launch whiteboat_navigation navigation.launch.py
```

Ou especificamente para a trajetória em quadrado:

```bash
ros2 launch whiteboat_navigation square.launch.py
```