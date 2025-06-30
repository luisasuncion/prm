# PRM - Programação de Robôs Móveis

# 🤖 Trabalho 1 - Sistema de Navegação com ROS 2

**Disciplina:** SSC0712 - Programação de Robôs Móveis  
**Professor:** Dr. Matheus Machado dos Santos  
**Grupo:** 5  
**Membros:** 
- Luis Enrique Asuncion Velasquez
- Ari Manuel Gamboa Aguilar
- Sandro Fabrizio Cárdenas Vilca

## 📋 Descrição do Projeto

Desenvolvimento de um sistema autônomo de navegação e controle de missão para um robô móvel no ambiente de simulação Gazebo, utilizando ROS 2.
O robô é capaz de:

- Explorar o ambiente.
- Detectar uma bandeira utilizando visão computacional.
- Navegar até a bandeira evitando obstáculos.
- Posicionar-se adequadamente para a coleta.

A arquitetura do sistema é baseada em uma máquina de estados implementada com ROS 2, sensores simulados e controle de movimento diferencial.

## 🧩 Máquina de Estados

- **EXPLORANDO**: movimento aleatório, evita obstáculos.
- **BANDEIRA_DETECTADA**: bandeira identificada, calcula posição relativa.
- **NAVIGANDO_PARA_BANDEIRA**: desloca-se até a bandeira desviando de obstáculos.
- **POSICIONANDO_PARA_COLETA**: ajusta posição e orientação.
- **MISSÃO_COMPLETA**: parada após o alinhamento correto.

> Toda a lógica de estados está implementada no script controle_robo.py utilizando sensor_msgs e geometry_msgs.

## 📦 Tecnologias utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- RViz
- OpenCV

---

## 🚀 Como utilizar o pacote

### ❗ Requisitos
- Sistema operacional Linux (Ubuntu 22.04 recomendado).
- ROS 2 Humble.
- Gazebo Fortress.
- **Dependências ROS 2**: ros_gz_bridge, ros_gz_sim, ros2_control, entre outras.

### 1. Clonar o repositório

Dentro da pasta `src` do seu workspace ROS 2:

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

> ⚠️ Importante: Execute previamente:
> ```bash
> sudo rosdep init
> rosdep update
> ```

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

## 🧪 Como executar a simulação
### ▶️ Se quiser executar tudo de uma vez:

```bash
ros2 launch prm missao_completa.launch.py
```
### ▶️ Se quiser executar passo a passo:

#### 1. Iniciar o mundo no Gazebo
```bash
ros2 launch prm inicia_simulacao.launch.py world:=empty_arena.sdf
```
#### 2. Carregar o robô no ambiente
Em um novo terminal (não se esqueça de `source install/local_setup.bash`):
```bash
ros2 launch prm carrega_robo.launch.py
```
#### 3. Controle automático
Em outro terminal:
```bash
ros2 launch prm executa_missao.launch.py
```

## Sensores Simulados

| Sensor     | Tópico         | Tipo de Mensagem         |
| ---------- | -------------- | ------------------------ |
| LIDAR      | `/scan`        | `sensor_msgs/LaserScan`  |
| IMU        | `/imu`         | `sensor_msgs/Imu`        |
| Odometria  | `/odom`        | `nav_msgs/Odometry`      |
| Câmera RGB | `/robot_cam`   | `sensor_msgs/Image`      |




