# Whiteboat Sensors

Este repositório contém os pacotes ROS 2 para o projeto Whiteboat, um sistema de navegação autônoma para barcos. Os pacotes incluem drivers para sensores, comunicação MAVLink, localização e navegação.

## Pacotes Incluídos

- **whiteboat_core**: Nó principal MAVLink, serviços e ferramentas básicas.
- **whiteboat_cam**: Driver de câmera (real e simulação VRX).
- **whiteboat_localization**: Fusão de dados GPS + IMU para localização.
- **whiteboat_navigation**: Controle de trajetória e navegação autônoma.

## Pré-requisitos

- **ROS 2 Humble Hawksbill** (ou versão compatível)
- **Ubuntu 22.04** (recomendado)
- **Python 3.10+**
- Dependências do sistema:
  - `python3-pymavlink`
  - `python3-serial`
  - `python3-opencv`
  - MAVROS
  - CV Bridge

### Instalação de Dependências

1. Instale ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. Instale dependências Python:
   ```bash
   pip install pymavlink pyserial opencv-python
   ```

3. Instale MAVROS:
   ```bash
   sudo apt install ros-humble-mavros ros-humble-mavros-msgs
   ```

4. Para simulação VRX (opcional):
   - Clone e instale o repositório VRX em `src/vrx/`

## Build e Instalação

1. Navegue para o workspace:
   ```bash
   cd /home/fernando/ros2_ws
   ```

2. Source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build os pacotes:
   ```bash
   colcon build --symlink-install
   ```

4. Source o workspace:
   ```bash
   source install/setup.bash
   ```

## Execução

### Modo Simulação (VRX)

> **Nota importante sobre o simulador:** Nas versões modernas do VRX para ROS 2 Humble (utilizando Gazebo Sim 7 / Garden), o ambiente base "vazio" com água não recebe mais o nome de `ocean`. O ambiente agora utilizado (carregado por padrão) é o `sydney_regatta`. Além disso, o pacote simulador foi corrigido para forçar a inicialização na versão 7 (`gz_version: '7'`) para prevenir conflitos de bibliotecas (`PluginHooks`).

1. **Terminal 1: Inicie o ambiente do simulador VRX e o barco** (carrega o ambiente e injeta o modelo):
   Abra um terminal, certifique-se de que o ROS está em source e exporte as texturas do Gazebo:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/vrx_gz/share:~/ros2_ws/install/wamv_description/share:~/ros2_ws/install/wamv_gazebo/share
   export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH}
   ros2 launch vrx_gz competition.launch.py
   ```

2. **Terminal 2: Inicie o ArduPilot SITL** (Cérebro do barco):
   Em outro terminal, acesse a ferramenta via seu profile e instancie a controladora visualizando-a sem abrir janelas de terminal extra:
   ```bash
   source ~/.profile
   sim_vehicle.py -v Rover -f JSON --out=udp:127.0.0.1:14550 --console
   ```
   *(Observação: Deixe essa instância aberta rodando, ela carregará os dados de sensores para os barcos).*

3. **Terminal 3: Lance o sistema Whiteboat via MAVROS**:
   Apenas quando os dois componentes vitais listados acima passarem dos seus estágios iniciais de "Waiting", abra um seu terceiro e último terminal (também não esqueça do seu `source` de rotina), e rode o driver conectável em modo simulação:
   ```bash
   ros2 launch whiteboat_core whiteboat.launch.py sim:=true
   ```

### Modo Hardware Real

1. Conecte o hardware (Pixhawk via USB).

2. Lance o sistema Whiteboat:
   ```bash
   ros2 launch whiteboat_core whiteboat.launch.py sim:=false
   ```

### Navegação Autônoma (Quadrado)

Execute o nó de navegação em quadrado:
```bash
ros2 run whiteboat_navigation square.py
```

## Dependências dos Pacotes

### whiteboat_core
- **ROS 2**: rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs
- **MAVLink**: mavros, mavros_msgs, pymavlink, serial
- **Outros**: tf2_ros, tf_transformations, rosidl_default_generators

### whiteboat_cam
- **ROS 2**: rclpy, sensor_msgs, std_msgs
- **Visão**: cv_bridge, opencv-python
- **MAVLink**: mavros_msgs

### whiteboat_localization
- **ROS 2**: rclpy, sensor_msgs, geographic_msgs
- **Dependências**: whiteboat_core, mavros, mavros_msgs, tf2_ros
- **Ferramentas**: ros2bag, rosbag2_transport

### whiteboat_navigation
- **ROS 2**: rclpy, sensor_msgs
- **Dependências**: whiteboat_core, mavros, mavros_msgs, tf2_ros, tf_transformations
- **Ferramentas**: ros2bag, rosbag2_transport

## Pendências e Problemas Conhecidos

- Licença: TODO (definir licença apropriada)
- Documentação: READMEs individuais dos pacotes incompletos
- Testes: Falta suíte de testes automatizados
- Configuração: Parâmetros de launch podem precisar ajuste para hardware específico
- Dependências: Verificar compatibilidade com versões mais recentes do ROS 2

## Contribuição

Para contribuir:
1. Fork o repositório
2. Crie uma branch para sua feature
3. Commit suas mudanças
4. Push para a branch
5. Abra um Pull Request

## Suporte

Para questões ou bugs, abra uma issue no repositório. 
