#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectorBandeira(Node):

    def __init__(self):
        super().__init__('detector_bandeira')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/robot_cam',
            self.camera_callback,
            10
        )

        self.pub_detectado = self.create_publisher(
            String,
            '/bandeira_detectada',
            10
        )

        self.get_logger().info("Detector de bandeira iniciado.")

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        # PROCESSAMENTO DE VISÃO - Exemplo com cor vermelha
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Faixa de cor vermelha no HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Dentro do if contours:
        if contours and cv2.contourArea(max(contours, key=cv2.contourArea)) > 500:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                w = cv_image.shape[1]
                pos_norm = cx / w  # Normalizado entre 0 y 1
                self.pub_detectado.publish(String(data=f"detected:{pos_norm:.2f}"))


        # DEBUG:
        # cv2.imshow("Visão", cv_image)
        # cv2.imshow("Máscara", mask)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorBandeira()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
