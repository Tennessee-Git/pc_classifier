import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from pc_c.Utils import convert_to_array
from pc_c.Constants import CLASSIFIED_IMAGE_TOPIC
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Display2DNode(Node):
    def __init__(self):
        super().__init__("display2d")
        self.subscription = self.create_subscription(
            Image,
            CLASSIFIED_IMAGE_TOPIC,
            self.image_callback,
            10
        )
        self.br = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Image received.")
        frame = self.br.imgmsg_to_cv2(msg)
        cv2.imshow("Classifed image", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    display = Display2DNode()
    rclpy.spin(display)

    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
