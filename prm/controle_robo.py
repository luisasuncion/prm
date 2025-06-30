#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from tf_transformations import euler_from_quaternion
from math import pi, sqrt, atan2

DISTANCIA_OBSTACULO = 0.4
POS_CENTRAL = 0.6
DISTANCIA_COLETA = 1.5
DISTANCIA_BASE = 0.5
TOLERANCIA_YAW = 0.15

class ControleRobo(Node):
    def __init__(self):
        super().__init__('controle_robo')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/bandeira_detectada', self.bandeira_callback, 10)

        self.timer = self.create_timer(0.1, self.move_robot)

        self.estado = 'EXPLORANDO'
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

        self.estado_retornar = 'NORMAL'
        self.contador_avanco = 0

        self.aguardando_animacao = False

    def finalizar_coleta(self):
        self.aguardando_animacao = False
        self.estado = 'RETORNANDO_PARA_BASE'
        self.get_logger().info("✅ Animação concluída. Iniciando retorno para base.")

    def scan_callback(self, msg):
        self.lidar_data = msg.ranges
        frente = msg.ranges[0:30] + msg.ranges[330:360]

        #esquerda = msg.ranges[60:120]
        #direita = msg.ranges[240:300]

        #self.get_logger().info(
        #    f"🚦 LIDAR: frente min={min(frente):.2f}, "
        #     f"esquerda min={min(esquerda):.2f}, "
        #    f"direita min={min(direita):.2f}"
        #)

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
                if len(partes) >= 3:
                    self.area_bandeira = float(partes[2])
                else:
                    self.area_bandeira = 0.0
                if self.estado == 'EXPLORANDO':
                    self.estado = 'BANDEIRA_DETECTADA'
            except ValueError:
                self.get_logger().warn("Formato inválido em bandeira_detectada.")

    def evitar_obstaculo(self):
        if not self.lidar_data or len(self.lidar_data) < 360:
            return -0.45  # fallback

        # faixa de 45° à esquerda e 45° à direita
        setor_esquerda = [
            d for d in self.lidar_data[45:90] if d != float('inf') and d > 0.0
        ]
        setor_direita = [
            d for d in self.lidar_data[270:315] if d != float('inf') and d > 0.0
        ]

        media_e = sum(setor_esquerda) / len(setor_esquerda) if setor_esquerda else 0
        media_d = sum(setor_direita) / len(setor_direita) if setor_direita else 0

        if media_d > media_e:
            return -0.5  # gira para direita
        else:
            return +0.5  # gira para esquerda



    def get_distancia_bandeira(self):
        if self.area_bandeira > 200:
            K = 20000.0
            return K / self.area_bandeira
        else:
            if not self.lidar_data or len(self.lidar_data) < 360:
                return float('inf')
            idx = int(self.bandeira_pos * 359)
            vizinhos = [
                d for i in range(-20, 21)
                if (0 <= (idx + i) % 360 < len(self.lidar_data))
                and not (d := self.lidar_data[(idx + i) % 360]) in [float('inf'), float('-inf')]
                and d > 0.0
            ]
            return min(vizinhos) if vizinhos else float('inf')

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def move_robot(self):
        twist = Twist()

        if self.estado == 'EXPLORANDO':
            self.get_logger().info("🔍 Estado: EXPLORANDO")

            if self.lidar_data and len(self.lidar_data) >= 360:
                # faixa da esquerda (60 a 120 graus)
                esquerda = [
                    d for d in self.lidar_data[60:120]
                    if not d in [float('inf'), float('-inf')] and d > 0.0
                ]

                # faixa da frente (para evitar colisões)
                frente = [
                    d for d in self.lidar_data[0:30] + self.lidar_data[330:360]
                    if not d in [float('inf'), float('-inf')] and d > 0.0
                ]

                dist_esquerda = min(esquerda) if esquerda else float('inf')
                dist_frente = min(frente) if frente else float('inf')

                #self.get_logger().info(f"🚦 LIDAR: esquerda min={dist_esquerda:.2f}, frente min={dist_frente:.2f}")

                distancia_desejada = 0.3  # distância alvo da parede
                erro_dist = distancia_desejada - dist_esquerda

                k_p = 1.0  # ganho proporcional
                ajuste = - k_p * erro_dist   # sinal invertido para seguir parede
                ajuste = max(min(ajuste, +0.8), -0.8)  # limitar ajuste para não ficar muito brusco

                if dist_frente < DISTANCIA_OBSTACULO:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5  # prioriza virar para direita
                    #self.get_logger().info("⚠️ Obstáculo à frente - desviando à direita.")
                elif dist_esquerda < 1.5:
                    # parede localizada, segue ajustando
                    twist.linear.x = 0.5
                    twist.angular.z = ajuste
                    #self.get_logger().info(f"🚗 Seguindo parede esquerda | erro={erro_dist:.2f} ajuste={ajuste:.2f}")
                else:
                    # sem parede detectada, segue reto
                    twist.linear.x = 0.25
                    twist.angular.z = 0.5
                    #self.get_logger().info("🔄 Sem parede à esquerda, avançando reto.")
            else:
                # dados lidar indisponíveis
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                #self.get_logger().warn("⚠️ Dados do LIDAR indisponíveis - avançando devagar.")

        elif self.estado == 'BANDEIRA_DETECTADA':
            self.get_logger().info("🎯 Estado: BANDEIRA_DETECTADA")
            self.estado = 'NAVIGANDO_PARA_BANDEIRA'

        elif self.estado == 'NAVIGANDO_PARA_BANDEIRA':
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
            twist.linear.x = 0.30 if abs(erro) < 0.1 else 0.0

            self.get_logger().info(f"Bandeira_pos: {self.bandeira_pos:.2f} | Erro: {erro:.2f} | Distância: {distancia:.2f} m")

            if distancia < DISTANCIA_COLETA:
                self.estado = 'POSICIONANDO_PARA_COLETA'
                msg = Float64MultiArray()
                msg.data = [2.0, 0.6, -0.6]
                self.gripper_pub.publish(msg)

        elif self.estado == 'POSICIONANDO_PARA_COLETA':
            self.get_logger().info("🤖 Estado: POSICIONANDO_PARA_COLETA")
            distancia = self.get_distancia_bandeira()

            self.get_logger().info(f"Distância Coleta: {distancia:.2f} m")

            if self.aguardando_animacao:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("⏳ Esperando animação da garra...")
                self.cmd_vel_pub.publish(twist)
                return

            if distancia >= 0.9:
                twist.linear.x = 0.6
            else:
                twist.linear.x = 2.0
                twist.angular.z = 0.0
                msg = Float64MultiArray()
                msg.data = [-2.0, 0.0, 0.0]
                self.gripper_pub.publish(msg)
                self.aguardando_animacao = True
                self.create_timer(10.0, self.finalizar_coleta)
                self.get_logger().info("🚩 Bandeira capturada. Aguardando animação...")
                self.estado = 'RETORNANDO_PARA_BASE'

        elif self.estado == 'RETORNANDO_PARA_BASE':
            self.get_logger().info("🏁 Estado: RETORNANDO_PARA_BASE")
            dx = self.x_base - self.x
            dy = self.y_base - self.y
            dist = sqrt(dx**2 + dy**2)
            ang_objetivo = atan2(dy, dx)
            erro_yaw = self.normalize_angle(ang_objetivo - self.yaw - pi/2)

            if dist > DISTANCIA_BASE:
                if self.estado_retornar == 'NORMAL':
                    if abs(erro_yaw) > TOLERANCIA_YAW:
                        twist.linear.x = 0.0
                        twist.angular.z = erro_yaw * 0.8
                        self.get_logger().info("🔄 Corrigindo orientação para a base...")
                    else:
                        if self.obstaculo_a_frente:
                            self.estado_retornar = 'DESVIANDO'
                            self.contador_avanco = 30
                            self.get_logger().info("⚠️ Obstáculo no retorno, iniciando desvio...")
                        else:
                            twist.linear.x = 0.45
                            twist.angular.z = 0.0
                            self.get_logger().info("⬆️ Avançando para a base.")

                elif self.estado_retornar == 'DESVIANDO':
                    twist.linear.x = 0.0
                    twist.angular.z = self.evitar_obstaculo()
                    self.contador_avanco = 30
                    self.estado_retornar = 'AVANCANDO'

                elif self.estado_retornar == 'AVANCANDO':
                    if self.obstaculo_a_frente:
                        twist.linear.x = 0.0
                        twist.angular.z = self.evitar_obstaculo()
                        self.estado_retornar = 'DESVIANDO'
                        self.get_logger().info("⚠️ Obstáculo detectado novamente durante avanço, retomando desvio...")
                    elif self.contador_avanco > 0:
                        twist.linear.x = 0.5
                        twist.angular.z = 0.0
                        self.contador_avanco -= 1
                        self.get_logger().info("↔️ Avançando após desvio...")
                    else:
                        self.estado_retornar = 'NORMAL'
                        self.get_logger().info("♻️ Recalculando rota para a base...")

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                msg = Float64MultiArray()
                msg.data = [0.0, 0.6, -0.6]
                self.gripper_pub.publish(msg)
                self.estado = 'FINALIZADO'
                self.get_logger().info("✅ Missão completa, bandeira entregue na base.")

        elif self.estado == 'FINALIZADO':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("🏆 Missão encerrada.")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
