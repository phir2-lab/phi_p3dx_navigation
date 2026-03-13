# phi_p3dx_navigation

Presente repositório contém pacotes para simulação do pioneer3dx no gazebo 2D, mobilesim 2D e robô real.

Ao clonar o repositório, é necessário compilar os pacotes com o colcon.

```bash
chmod +x src/phi_p3dx_navigation/scripts/*.py
```

```bash
cd pioneer3dx_ws
colcon build --symlink-install
source install/setup.bash ou install/setup.sh ou install/setup.zsh
```

Para visualizar o robô no rviz:

```bash
ros2 launch model_bringup bringup_rviz.launch.py
```
As vezes, é necessário mudar o "fixed frame" para "base_link" se estiver com "odom" no rviz.

Para simular o robô no mobilesim 2D:

```bash
ros2 launch model_bringup bringup_mobilesim.launch.py
```

Para simular o robô no gazebo 3D:

```bash
ros2 launch model_bringup bringup_gazebo.launch.py
```

Para simular o robô no robô real:

```bash
ros2 launch model_bringup bringup_real.launch.py
```
