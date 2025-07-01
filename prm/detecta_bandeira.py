#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Importa tipos de mensagens ROS
from sensor_msgs.msg import Image
from std_msgs.msg import String

# cv_bridge faz a ponte entre ROS e OpenCV
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectorBandeira(Node):
    """
    Nó ROS2 que faz a detecção de uma bandeira
    a partir da imagem colorida da câmera do robô,
    segmentando por cor e enviando a posição da bandeira detectada.
    """

    def __init__(self):
        # Inicializa o nó com o nome 'detector_bandeira'
        super().__init__('detector_bandeira')

        self.bridge = CvBridge()  # cria a ponte para converter mensagens de imagem ROS para OpenCV

        # Subscreve o tópico de imagem da câmera do robô
        self.subscription = self.create_subscription(
            Image,
            '/robot_cam/colored_map',  # tópico de entrada de imagem
            self.camera_callback,      # função callback
            10                         # fila de mensagens
        )

        # Publicador para enviar a mensagem de detecção
        self.pub_detectado = self.create_publisher(
            String,
            '/bandeira_detectada',     # tópico de saída
            10
        )

        self.get_logger().info("📷 Detector de bandeira com segmentação iniciado.")

    def camera_callback(self, msg):
        """
        Callback chamado sempre que chega uma nova imagem da câmera
        """
        try:
            # Converte a imagem ROS para formato OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        # Obtém altura e largura da imagem
        h, w, _ = cv_image.shape

        # Lê o pixel central (opcional para debug)
        pixel = cv_image[h // 2, w // 2]
        b, g, r = int(pixel[0]), int(pixel[1]), int(pixel[2])

        # Cria uma máscara segmentando exatamente a cor (227, 73, 0) pode ser ajustado com tolerância usando um range maior
        mask = cv2.inRange(cv_image, (227, 73, 0), (227, 73, 0))

        # Visualização opcional:
        # cv2.imshow("Máscara", mask)
        # cv2.imshow("Visão", cv_image)
        # cv2.waitKey(1)

        # Procura contornos na máscara para encontrar regiões da bandeira
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #self.get_logger().info(f"🔍 Contornos encontrados: {len(contours)}")

        if contours:
            # Seleciona o maior contorno
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            #self.get_logger().info(f"📏 Área do maior contorno: {area:.2f}")
            if area > 200:  # ignora contornos muito pequenos (ruído)
                # Calcula o centroide do contorno
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])  # coordenada x do centroide
                    pos_norm = cx / w  # posição normalizada de 0 a 1
                    # Publica a posição e área no tópico
                    self.pub_detectado.publish(String(data=f"detected:{pos_norm:.2f}:{area:.0f}"))
                    #self.get_logger().info(f"📍 Bandeira detectada! Posição normalizada: {pos_norm:.2f}")

def main(args=None):
    """
    Função principal ROS2
    """
    rclpy.init(args=args)
    node = DetectorBandeira()
    rclpy.spin(node)         # executa o nó até encerrar
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
