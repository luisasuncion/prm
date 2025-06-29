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
            '/robot_cam/colored_map',
            self.camera_callback,
            10
        )

        self.pub_detectado = self.create_publisher(
            String,
            '/bandeira_detectada',
            10
        )

        self.get_logger().info("📷 Detector de bandeira com segmentação iniciado.")

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        h, w, _ = cv_image.shape
        pixel = cv_image[h // 2, w // 2]
        b, g, r = int(pixel[0]), int(pixel[1]), int(pixel[2])
        # self.get_logger().info(f"🎨 Pixel central: R={r} G={g} B={b}")

        mask = cv2.inRange(cv_image, (227, 73, 0), (227, 73, 0))
        #cv2.imshow("Máscara", mask)
        #cv2.imshow("Visão", cv_image)
        #cv2.waitKey(1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #self.get_logger().info(f"🔍 Contornos encontrados: {len(contours)}")

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            self.get_logger().info(f"📏 Área do maior contorno: {area:.2f}")
            if area > 200:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    pos_norm = cx / w
                    self.pub_detectado.publish(String(data=f"detected:{pos_norm:.2f}:{area:.0f}"))
                    #self.get_logger().info(f"📍 Bandeira detectada! Posição normalizada: {pos_norm:.2f}")




def main(args=None):
    rclpy.init(args=args)
    node = DetectorBandeira()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
