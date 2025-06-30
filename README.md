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

esenvolvimento de um sistema autônomo de navegação e coleta de objetos para um robô móvel no ambiente de simulação Gazebo, utilizando ROS 2.

O robô é capaz de:

✅ Explorar o ambiente evitando obstáculos
✅ Detectar uma bandeira via visão computacional (mensagens String simuladas)
✅ Navegar até a bandeira ajustando sua posição
✅ Realizar a coleta simulada da bandeira, acionando uma garra virtual
✅ Retornar automaticamente à base inicial
✅ Entregar a bandeira e finalizar a missão

O controle é estruturado por meio de máquina de estados gerenciada pelo script controle_robo.py, usando mensagens ROS 2 padrão (sensor_msgs, geometry_msgs, std_msgs).

## 🧩 Máquina de Estados

- **EXPLORANDO**  
  Explora o ambiente, contornando paredes e evitando obstáculos via LIDAR.

- **BANDEIRA_DETECTADA**  
  Detecta a bandeira através de mensagem recebida, passando para navegação direcionada.

- **NAVIGANDO_PARA_BANDEIRA**  
  Ajusta orientação e desloca-se até a bandeira, evitando obstáculos durante a aproximação.

- **POSICIONANDO_PARA_COLETA**  
  Finaliza a aproximação da bandeira, aciona a garra virtual, e aguarda a animação de coleta.

- **RETORNANDO_PARA_BASE**  
  Navega de volta ao ponto de partida (base), com correção de orientação e desvio de obstáculos, até soltar a bandeira.

- **FINALIZADO**  
  Final da missão, bandeira entregue na base.

> Toda a lógica de estados está implementada no script controle_robo.py utilizando sensor_msgs e geometry_msgs.

## 📦 Tecnologias utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- RViz
- OpenCV

---

## ⚙️ Parâmetros de Controle

- **DISTANCIA_OBSTACULO**: 0.4 m  
- **POS_CENTRAL**: 0.6  
- **DISTANCIA_COLETA**: 1.5 m  
- **DISTANCIA_BASE**: 0.5 m  
- **TOLERANCIA_YAW**: 0.15 rad  

Estes parâmetros podem ser ajustados diretamente no código-fonte.

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

#### 1. Iniciar o mundo no Gazebo
```bash
ros2 launch prm inicia_simulacao.launch.py world:=empty_arena.sdf
```

```bash
ros2 launch prm inicia_simulacao.launch.py world:=arena_cilindros.sdf
```

#### 2. Carregar o robô no ambiente
Em um novo terminal (não se esqueça de `source install/local_setup.bash`):
```bash
ros2 launch prm carrega_robo.launch.py
```
#### 3. Controle automatico

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

## 📈 Fluxo da Missão

1️⃣ Explora o ambiente seguindo paredes e evitando obstáculos
2️⃣ Detecta a bandeira via /bandeira_detectada
3️⃣ Navega até a bandeira ajustando orientação e distância
4️⃣ Aciona a garra virtual para coleta e aguarda a animação
5️⃣ Retorna à base evitando obstáculos
6️⃣ Entrega a bandeira e finaliza a missão