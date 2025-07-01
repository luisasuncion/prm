#!/usr/bin/env python3
"""
Script de controle do robô ROS 2 para exploração, detecção e coleta de bandeiras
com retorno automático à base, baseado em máquina de estados.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray

from tf_transformations import euler_from_quaternion

from math import pi, sqrt, atan2
from enum import Enum

# -------------------------------
# Constantes de controle
# -------------------------------
DISTANCIA_OBSTACULO = 0.4
POS_CENTRAL = 0.6
DISTANCIA_COLETA = 1.5
DISTANCIA_BASE = 0.5
DISTANCIA_ENTRADA_BASE = 1.5
TOLERANCIA_YAW = 0.15
K_DISTANCIA_AREA = 20000.0
AJUSTE_MAX = 0.8

# -------------------------------
# Enum de estados
# -------------------------------
class Estado(str, Enum):
    EXPLORANDO = 'EXPLORANDO'
    BANDEIRA_DETECTADA = 'BANDEIRA_DETECTADA'
    NAVIGANDO_PARA_BANDEIRA = 'NAVIGANDO_PARA_BANDEIRA'
    POSICIONANDO_PARA_COLETA = 'POSICIONANDO_PARA_COLETA'
    RETORNANDO_PARA_BASE = 'RETORNANDO_PARA_BASE'
    FINALIZADO = 'FINALIZADO'

class ControleRobo(Node):
    """
    Nó de controle principal do robô
    """
    def __init__(self):
        super().__init__('controle_robo')

        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        # Assinantes
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/bandeira_detectada', self.bandeira_callback, 10)

        # Timer de controle
        self.timer = self.create_timer(0.1, self.move_robot)

        # Variáveis de estado
        self.estado = Estado.EXPLORANDO
        self.estado_retornar = 'NORMAL'

        self.obstaculo_a_frente = False
        self.bandeira_pos = 0.5
        self.area_bandeira = 0.0
        self.lidar_data = []
        self.direcao_evasao = None

        self.yaw = 0.0
        self.yaw_inicial = None
        self.x = 0.0
        self.y = 0.0

        self.x_base = -8.0
        self.y_base = -0.5

        self.contador_giro = 0

    # ---------------------------------------------------
    # Callbacks de sensores
    # ---------------------------------------------------
    def scan_callback(self, msg):
        self.lidar_data = msg.ranges
        frente = msg.ranges[0:30] + msg.ranges[330:360]
        frente_valido = [d for d in frente if d != float('inf')]
        self.obstaculo_a_frente = min(frente_valido) < DISTANCIA_OBSTACULO if frente_valido else False

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw_leido = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if self.yaw_inicial is None:
            self.yaw_inicial = yaw_leido
        self.yaw = self.normalize_angle(yaw_leido - self.yaw_inicial)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw_leido = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if self.yaw_inicial is None:
            self.yaw_inicial = yaw_leido
        self.yaw = self.normalize_angle(yaw_leido - self.yaw_inicial)

    def bandeira_callback(self, msg):
        if msg.data.startswith("detected:"):
            try:
                partes = msg.data.split(":")
                self.bandeira_pos = float(partes[1])
                self.area_bandeira = float(partes[2]) if len(partes) >= 3 else 0.0
                if self.estado == Estado.EXPLORANDO:
                    self.estado = Estado.BANDEIRA_DETECTADA
            except Exception as e:
                self.get_logger().warn(f"Erro no formato da bandeira_detectada: {e}")

    def evitar_obstaculo(self):
        """
        Gera comando angular para desvio de obstáculos
        """
        if not self.lidar_data or len(self.lidar_data) < 360:
            return -0.45  # fallback

        # calcula médias laterais
        setor_esquerda = [
            d for d in self.lidar_data[45:90] if d != float('inf') and d > 0.0
        ]
        setor_direita = [
            d for d in self.lidar_data[270:315] if d != float('inf') and d > 0.0
        ]

        media_e = sum(setor_esquerda) / len(setor_esquerda) if setor_esquerda else 0
        media_d = sum(setor_direita) / len(setor_direita) if setor_direita else 0

        # decide sentido do giro
        if media_d > media_e:
            return -0.5  # gira para direita
        else:
            return +0.5  # gira para esquerda
        
    # ---------------------------------------------------
    # Utilitários
    # ---------------------------------------------------
    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def controla_gripper(self, elevacao, esquerda, direita):
        msg = Float64MultiArray()
        msg.data = [elevacao, esquerda, direita]
        self.gripper_pub.publish(msg)

    def get_distancia_bandeira(self):
        if self.area_bandeira > 200:
            return K_DISTANCIA_AREA / self.area_bandeira
        else:
            if not self.lidar_data or len(self.lidar_data) < 360:
                return float('inf')
            idx = int(self.bandeira_pos * 359)
            vizinhos = [
                d for i in range(-20, 21)
                if 0 <= (idx + i) % 360 < len(self.lidar_data)
                and not (d := self.lidar_data[(idx + i) % 360]) in [float('inf'), float('-inf')]
                and d > 0.0
            ]
            return min(vizinhos) if vizinhos else float('inf')

    def seguir_parede(self):
        esquerda = [
            d for d in self.lidar_data[60:120]
            if d not in [float('inf'), float('-inf')] and d > 0.0
        ]
        frente = [
            d for d in self.lidar_data[0:30] + self.lidar_data[330:360]
            if d not in [float('inf'), float('-inf')] and d > 0.0
        ]
        dist_esquerda = min(esquerda) if esquerda else float('inf')
        dist_frente = min(frente) if frente else float('inf')

        erro_dist = 0.3 - dist_esquerda
        ajuste = -erro_dist
        ajuste = max(min(ajuste, +AJUSTE_MAX), -AJUSTE_MAX)

        twist = Twist()
        if dist_frente < DISTANCIA_OBSTACULO:
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        elif dist_esquerda < 1.5:
            twist.linear.x = 0.5
            twist.angular.z = ajuste
        else:
            twist.linear.x = 0.25
            twist.angular.z = 0.5
        return twist

    # ---------------------------------------------------
    # Máquina de estados
    # ---------------------------------------------------
    def move_robot(self):
        twist = Twist()

        # ---------------------------
        # EXPLORANDO
        # ---------------------------
        if self.estado == Estado.EXPLORANDO:
            self.get_logger().info("🔍 Estado: EXPLORANDO")
            twist = self.seguir_parede()

        # ---------------------------
        # BANDEIRA_DETECTADA
        # ---------------------------
        elif self.estado == Estado.BANDEIRA_DETECTADA:
            self.get_logger().info("🎯 Estado: BANDEIRA_DETECTADA")
            self.estado = Estado.NAVIGANDO_PARA_BANDEIRA

        # ---------------------------
        # NAVIGANDO_PARA_BANDEIRA
        # ---------------------------
        elif self.estado == Estado.NAVIGANDO_PARA_BANDEIRA:
            self.get_logger().info("🚗 Estado: NAVIGANDO_PARA_BANDEIRA")
            erro = self.bandeira_pos - POS_CENTRAL
            distancia = self.get_distancia_bandeira()
            giro_bandeira = -erro * 1.5
            if self.obstaculo_a_frente:
                twist.linear.x = 0.0
                twist.angular.z = self.evitar_obstaculo()
                self.cmd_vel_pub.publish(twist)
                return
            twist.angular.z = giro_bandeira
            twist.linear.x = 0.3 if abs(erro) < 0.1 else 0.0

            if distancia < DISTANCIA_COLETA:
                self.estado = Estado.POSICIONANDO_PARA_COLETA
                self.controla_gripper(2.0, 0.6, -0.6)

        # ---------------------------
        # POSICIONANDO_PARA_COLETA
        # ---------------------------
        elif self.estado == Estado.POSICIONANDO_PARA_COLETA:
            self.get_logger().info("🤖 Estado: POSICIONANDO_PARA_COLETA")
            distancia = self.get_distancia_bandeira()
            if distancia >= 0.9:
                twist.linear.x = 0.6
            else:
                twist.linear.x = 2.0
                self.controla_gripper(-2.0, 0.0, 0.0)
                self.estado = Estado.RETORNANDO_PARA_BASE
                self.yaw_giro_inicial = self.yaw
                self.estado_retornar = 'GIRO_INICIAL'

        # ---------------------------
        # RETORNANDO_PARA_BASE
        # ---------------------------
        elif self.estado == Estado.RETORNANDO_PARA_BASE:
            dx = self.x_base - self.x
            dy = self.y_base - self.y
            dist_base = sqrt(dx**2 + dy**2)

            if self.estado_retornar == 'GIRO_INICIAL':
                if self.contador_giro < 50:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                    self.contador_giro += 1
                else:
                    self.estado_retornar = 'SEGUINDO_PAREDE'
                    self.contador_giro = 0

            elif self.estado_retornar == 'SEGUINDO_PAREDE':
                if dist_base < DISTANCIA_ENTRADA_BASE:
                    self.estado_retornar = 'ENTRANDO_BASE'
                    self.get_logger().info("📌 Entrando na zona da base")
                else:
                    twist = self.seguir_parede()

            elif self.estado_retornar == 'ENTRANDO_BASE':
                ang_objetivo = atan2(dy, dx)
                erro_yaw = self.normalize_angle(ang_objetivo - self.yaw - pi/2)
                if dist_base > DISTANCIA_BASE:
                    if abs(erro_yaw) > TOLERANCIA_YAW:
                        twist.angular.z = erro_yaw * 0.8
                        twist.linear.x = 0.0
                    else:
                        if self.obstaculo_a_frente:
                            twist.linear.x = 0.0
                            twist.angular.z = self.evitar_obstaculo()
                        else:
                            twist.linear.x = 0.45
                            twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.controla_gripper(0.0, 0.6, -0.6)
                    self.estado = Estado.FINALIZADO

        # ---------------------------
        # FINALIZADO
        # ---------------------------
        elif self.estado == Estado.FINALIZADO:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("🏆 Missão encerrada!")

        # publicar comandos
        self.cmd_vel_pub.publish(twist)

# ------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
