# PRM - Programação de Robôs Móveis

# 🤖 Trabalho 1 - Sistema de Navegação e Controle da Missão com ROS 2

**Disciplina:** SSC0712 - Programação de Robôs Móveis  
**Professor:** Dr. Matheus Machado dos Santos  
**Grupo:** 5
**Membros do grupo:** 
- Luis Enrique Asuncion Velasquez
- Ari Manuel Gamboa Aguilar
- Sandro Fabrizio Cárdenas Vilca

## 📦 Tecnologias utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- RViz / Gazebo GUI
- OpenCV

---

## 🚀 Como utilizar o pacote

### 1. Clonar o repositório

Acesse a pasta `src` do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/luisasuncion/prm.git
````

### 2. Instalar dependências

Instale as dependências do pacote com:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Certifique-se de ter rodado previamente `sudo rosdep init` e `rosdep update`, se for a primeira vez usando o `rosdep`.

### 3. Compilar o workspace

Certifique-se de estar na **raiz do seu workspace** (geralmente `~/ros2_ws`) antes de compilar:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select prm
```

### 4. Atualizar o ambiente do terminal

```bash
source install/local_setup.bash
```

---

## 🧪 Executando a simulação

```bash
ros2 launch prm missao_completa.launch.py
```

## Sensores Simulados

| Sensor     | Tópico         | Tipo de Mensagem         |
| ---------- | -------------- | ------------------------ |
| LIDAR      | `/scan`        | `sensor_msgs/LaserScan`  |
| IMU        | `/imu`         | `sensor_msgs/Imu`        |
| Odometria  | `/odom`        | `nav_msgs/Odometry`      |
| Câmera RGB | `/robot_cam`   | `sensor_msgs/Image`      |
| CameraInfo | `/camera_info` | `sensor_msgs/CameraInfo` |





