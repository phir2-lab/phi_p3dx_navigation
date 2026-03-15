# phi_p3dx_navigation

Este repositório contém pacotes ROS2 para navegação autônoma do robô Pioneer P3-DX em diferentes ambientes: simulação no Gazebo (3D), MobileSim (2D) e robô real.

## Visão Geral

O pacote `phi_p3dx_navigation` fornece:
- **Nós de navegação reativos**: Implementações simples de controle para evitar obstáculos e alcançar objetivos.
- **Compatibilidade multi-plataforma**: Funciona em Gazebo, MobileSim e robô físico.
- **Exemplos educacionais**: Código comentado em Python e C++ para estudantes de robótica.

### Estrutura do Pacote
- `scripts/`: Scripts Python (ex.: `control_example.py`).
- `phi_p3dx_navigation/`: Classe base Python
- `src/`: Código C++ (ex.: `control_example.cpp`, `navigation_node.cpp`).
- `include/`: Headers C++ (ex.: `navigation_node.hpp`).
- `launch/`: Arquivos de launch para diferentes ambientes.
- `config/`: Configurações (ex.: RViz, mapas).

## Pré-requisitos

- **ROS2 Humble** (ou compatível).
- **Gazebo** (para simulação 3D).
- **MobileSim** (para simulação 2D, se disponível).
- Dependências: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`.

## Instalação

1. **Clone o repositório** no seu workspace ROS2:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_navigation.git
   ```

2. **Compile o pacote**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_navigation
   source install/setup.bash
   ```

## Uso

### Simulação no Gazebo (3D)

1. Lance a simulação:
   ```bash
   ros2 launch phi_p3dx_navigation bringup_gazebo.launch.py
   ```
2. Execute o nó de navegação (Python):
   ```bash
   ros2 run phi_p3dx_navigation control_example_py
   ```

   Ou em C++:
   ```bash
   ros2 run phi_p3dx_navigation control_example_cpp
   ```

3. No RViz:
   - Use "2D Nav Goal" para enviar objetivos.


### Simulação no MobileSim (2D)

1. Lance a simulação:
   ```bash
   ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py
   ```

2. Execute o nó de navegação conforme acima.

### Robô Real

1. Conecte o robô Pioneer P3-DX.

2. Lance o sistema:
   ```bash
   ros2 launch phi_p3dx_navigation bringup_real.launch.py
   ```

3. Execute o nó de navegação.

## Exemplos de Navegação

### Algoritmo Básico
- **Virar em direção ao objetivo**: Calcula ângulo de erro e gira.
- **Andar em linha reta**: Quando alinhado, avança.
- **Evitar obstáculos**: Para se detectar algo à frente (threshold: 0.5m).
- **Objetivo alcançado**: Para quando próximo o suficiente (threshold: 0.1m).

## Documentação

- **Python**: Docstrings podem ser visualizadas com `pydoc` ou Sphinx.
- **C++**: Comentários Doxygen. Gere docs com:
  ```bash
  cd src/phi_p3dx_navigation
  doxygen -g  # Cria Doxyfile
  doxygen     # Gera docs em html/
  ```

## Licença

Este projeto é distribuído sob a licença MIT.
