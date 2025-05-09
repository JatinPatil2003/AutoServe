#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_sub = self.create_subscription(
            Image, '/camera_2/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera_2/depth/image_raw', self.depth_callback, 10)
        self.bridge = CvBridge()
        self.cv_image = None
        self.cv_depth = None

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # print(self.cv_depth)

    def update_images(self):
        if self.cv_image is not None:
            cv2.imshow("Camera 2 RGB Image", self.cv_image)
        if self.cv_depth is not None:
            cv2.imshow("Camera 2 Depth Image", self.cv_depth)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    while rclpy.ok():
        rclpy.spin_once(image_subscriber)
        image_subscriber.update_images()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
