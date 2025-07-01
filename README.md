# PRM - Programação de Robôs Móveis

## 🤖 Trabalho Final — Sistema de Navegação Autônoma com ROS 2

**Disciplina:** SSC0712 — Programação de Robôs Móveis  
**Professor:** Dr. Matheus Machado dos Santos  
**Grupo:** 5  
**Membros:**  

- Luis Enrique Asuncion Velasquez  
- Ari Manuel Gamboa Aguilar  
- Sandro Fabrizio Cárdenas Vilca  

---

## 📋 Descrição do Projeto

Desenvolvimento de um sistema autônomo de navegação e coleta de objetos para um robô móvel no ambiente de simulação Gazebo, utilizando ROS 2.

O robô é capaz de:

- Explorar o ambiente evitando obstáculos  
- Detectar uma bandeira via visão computacional (mensagens String simuladas)  
- Navegar até a bandeira ajustando sua posição  
- Realizar a coleta simulada da bandeira, acionando uma garra virtual  
- Retornar automaticamente à base inicial  
- Entregar a bandeira e finalizar a missão

O controle é estruturado por meio de uma **máquina de estados** gerenciada pelo script `controle_robo.py`, utilizando mensagens padrão do ROS 2 (`sensor_msgs`, `geometry_msgs`, `std_msgs`).

---

## 🧩 Máquina de Estados

- **EXPLORANDO:** Explora o ambiente, contornando paredes e evitando obstáculos via LIDAR.
- **BANDEIRA_DETECTADA:** Detecta a bandeira através de mensagem recebida, iniciando navegação direcionada.
- **NAVIGANDO_PARA_BANDEIRA:** Ajusta orientação e desloca-se até a bandeira, evitando obstáculos durante a aproximação.
- **POSICIONANDO_PARA_COLETA:** Finaliza a aproximação, aciona a garra virtual e aguarda a coleta.
- **RETORNANDO_PARA_BASE:** Retorna ao ponto de partida, corrigindo orientação e desviando de obstáculos até soltar a bandeira.
- **FINALIZADO:** Missão concluída, bandeira entregue na base.

---

## 📦 Tecnologias Utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- RViz
- OpenCV

---

## ⚙️ Parâmetros de Controle

| Parâmetro             | Valor padrão |
|-----------------------|--------------|
| `DISTANCIA_OBSTACULO` | 0.4 m        |
| `POS_CENTRAL`         | 0.6          |
| `DISTANCIA_COLETA`    | 1.5 m        |
| `DISTANCIA_BASE`      | 0.5 m        |
| `TOLERANCIA_YAW`      | 0.15 rad     |

Esses valores podem ser ajustados diretamente no arquivo `controle_robo.py`.

---

## 🚀 Como Utilizar o Pacote

### ❗ Requisitos

- Linux (Ubuntu 22.04 recomendado)
- ROS 2 Humble
- Gazebo Fortress
- **Dependências ROS 2:** ros_gz_bridge, ros_gz_sim, ros2_control, entre outras

### 1. Clonar o Repositório

```bash
cd ~/ros2_ws/src/
git clone https://github.com/luisasuncion/prm.git
```

### 2. Instalar Dependências

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> ⚠️ Execute previamente:
> ```bash
> sudo rosdep init
> rosdep update
> ```

### 3. Compilar o Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select prm
```

### 4. Atualizar o Ambiente do Terminal

```bash
source install/local_setup.bash
```

---

## 🧪 Como Executar a Simulação

### 1. Iniciar o Mundo no Gazebo

Três opções de arenas:

- Arena padrão (cilindros):

  ```bash
  ros2 launch prm inicia_simulacao.launch.py world:=arena_cilindros.sdf
  ```
  
- Arena livre (vazia):

  ```bash
  ros2 launch prm inicia_simulacao.launch.py world:=empty_arena.sdf
  ```

- Arena com paredes (labirinto):

  ```bash
  ros2 launch prm inicia_simulacao.launch.py world:=arena_paredes.sdf
  ```

### 2. Carregar o Robô no Ambiente

Em um novo terminal (não se esqueça de `source install/local_setup.bash`):

```bash
ros2 launch prm carrega_robo.launch.py
```

### 3. Controle Automático

```bash
ros2 launch prm executa_missao.launch.py
```

---

## 🛰️ Sensores Simulados

| Sensor     | Tópico         | Tipo de Mensagem         |
| ---------- | -------------- | ------------------------ |
| LIDAR      | `/scan`        | `sensor_msgs/LaserScan`  |
| IMU        | `/imu`         | `sensor_msgs/Imu`        |
| Odometria  | `/odom`        | `nav_msgs/Odometry`      |
| Câmera RGB | `/robot_cam`   | `sensor_msgs/Image`      |

---

## 📈 Fluxo da Missão

1. Explora o ambiente seguindo paredes e evitando obstáculos  
2. Detecta a bandeira via `/bandeira_detectada`  
3. Navega até a bandeira ajustando orientação e distância  
4. Aciona a garra virtual para coleta e aguarda a animação  
5. Retorna à base evitando obstáculos  
6. Entrega a bandeira e finaliza a missão  

---

## 📁 Estrutura do Projeto

```
prm/
├── description/
│   └── r2d2.urdf.xacro           # Modelo do robô
├── launch/
│   ├── inicia_simulacao.launch.py
│   ├── carrega_robo.launch.py
│   ├── executa_missao.launch.py
│   └── editar_mundo.launch.py
├── scripts/
│   ├── controle_robo.py          # Máquina de estados
│   ├── detecta_bandeira.py       # Detecção da bandeira
│   ├── robo_mapper.py            # Mapeamento do grid
│   └── ground_truth_odometry.py  # Odometria ground truth
├── rviz/
│   └── rviz_config.rviz
├── world/
│   ├── arena_cilindros.sdf
│   ├── empty_arena.sdf
│   └── arena_obstaculos.sdf
├── package.xml
└── setup.py
```
