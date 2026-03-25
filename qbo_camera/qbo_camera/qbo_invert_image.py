#!/usr/bin/env python3

# inversion 180° image
# charge CPU = 7.9 %
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRotator(Node):
    def __init__(self):
        super().__init__('eye_image_rotation')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)
        self.publisher = self.create_publisher(Image, '/inverted_eye_image', 1)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        rotated_msg = self.bridge.cv2_to_imgmsg(rotated_image, "bgr8")
        self.publisher.publish(rotated_msg)

def main():
    rclpy.init()
    rotator = ImageRotator()
    rclpy.spin(rotator)

