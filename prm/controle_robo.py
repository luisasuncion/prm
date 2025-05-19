#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
from math import sqrt

DISTANCIA_OBSTACULO = 0.38
JANELA_LIDAR = 10
POS_CENTRAL = 0.7
DISTANCIA_COLETA = 0.3
TOLERANCIA_YAW = 0.1
DISTANCIA_OBJETIVO = 0.20

class ControleRobo(Node):
    def __init__(self):
        super().__init__('controle_robo')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/bandeira_detectada', self.bandeira_callback, 10)

        self.timer = self.create_timer(0.1, self.move_robot)

        self.estado = 'EXPLORANDO'
        self.obstaculo_a_frente = False
        self.bandeira_pos = 0.5
        self.lidar_data = []
        self.direcao_evasao = None

        self.x = self.y = self.yaw = 0.0
        self.x_objetivo = self.y_objetivo = self.yaw_objetivo = 0.0

    def scan_callback(self, msg):
        self.lidar_data = msg.ranges
        frente = msg.ranges[0:30] + msg.ranges[330:360]
        frente_valido = [d for d in frente if d != float('inf')]
        self.obstaculo_a_frente = min(frente_valido) < DISTANCIA_OBSTACULO if frente_valido else False

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def bandeira_callback(self, msg):
        if msg.data.startswith("detected:"):
            try:
                self.bandeira_pos = float(msg.data.split(":")[1])
                if self.estado == 'EXPLORANDO':
                    self.estado = 'BANDEIRA_DETECTADA'
            except ValueError:
                self.get_logger().warn("Formato inválido en bandeira_detectada.")

    def evitar_obstaculo(self):
        if not self.lidar_data or len(self.lidar_data) < 360:
            return 0.5

        esquerda = [d for d in self.lidar_data[60:120] if d != float('inf')]
        direita  = [d for d in self.lidar_data[240:300] if d != float('inf')]

        media_e = sum(esquerda) / len(esquerda) if esquerda else 0
        media_d = sum(direita) / len(direita) if direita else 0

        if self.direcao_evasao is None:
            self.direcao_evasao = 'esquerda' if media_e > media_d else 'direita'

        return 0.5 if self.direcao_evasao == 'esquerda' else -0.5

    def get_distancia_bandeira(self):
        if not self.lidar_data or len(self.lidar_data) < 360:
            return float('inf')

        idx = int(self.bandeira_pos * 359)
        dist = self.lidar_data[idx]

        if dist == float('inf'):
            vizinhos = [
                self.lidar_data[(idx + i) % 360]
                for i in range(-JANELA_LIDAR, JANELA_LIDAR + 1)
                if self.lidar_data[(idx + i) % 360] != float('inf')
            ]
            return min(vizinhos) if vizinhos else float('inf')

        return dist

    def move_robot(self):
        twist = Twist()

        if self.estado == 'EXPLORANDO':
            self.get_logger().info("🔍 Estado: EXPLORANDO")
            if self.obstaculo_a_frente:
                twist.linear.x = 0.0
                twist.angular.z = self.evitar_obstaculo()
            else:
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                self.direcao_evasao = None

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
                self.get_logger().info("⚠️ Obstáculo detectado. Evadiendo...")
                self.cmd_vel_pub.publish(twist)
                return

            twist.angular.z = giro_bandeira
            twist.linear.x = 0.12 if abs(erro) < 0.1 else 0.0

            self.get_logger().info(f"Bandeira_pos: {self.bandeira_pos:.2f} | Erro: {erro:.2f}")
            self.get_logger().info(f"Giro: {giro_bandeira:.2f} | Distância: {distancia:.2f} m")

            if distancia < DISTANCIA_COLETA and not self.obstaculo_a_frente:
                self.x_objetivo = self.x
                self.y_objetivo = self.y
                self.yaw_objetivo = self.yaw
                self.estado = 'POSICIONANDO_PARA_COLETA'

        elif self.estado == 'POSICIONANDO_PARA_COLETA':
            self.get_logger().info("🤖 Estado: POSICIONANDO_PARA_COLETA")
            dist = sqrt((self.x - self.x_objetivo) ** 2 + (self.y - self.y_objetivo) ** 2)
            erro_yaw = self.yaw - self.yaw_objetivo

            if dist > DISTANCIA_OBJETIVO:
                twist.linear.x = 0.05
            elif abs(erro_yaw) > TOLERANCIA_YAW:
                twist.linear.x = 0.0
                twist.angular.z = -erro_yaw
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("🚩 Frente à bandeira. Pronto para coleta.")

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
