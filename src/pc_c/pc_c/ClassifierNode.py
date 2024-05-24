import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pc_c_interfaces.msg import LidarData
from pc_c.Utils import convert_to_array
from pc_c.Constants import LIDAR_DATA_TOPIC, CLASSIFIED_IMAGE_TOPIC

import numpy as np


class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier')
        self.subscription = self.create_subscription(
            LidarData,
            LIDAR_DATA_TOPIC,
            self.classify_callback,
            10
        )

        self.publisher_ = self.create_publisher(Image, CLASSIFIED_IMAGE_TOPIC, 10)

    def classify_callback(self, msg):
        xyz = convert_to_array(msg.points)
        self.get_logger().info(f'RECEIVED {xyz.shape}')


def main(args=None):
    rclpy.init(args=args)
    classifier = ClassifierNode()
    rclpy.spin(classifier)

    classifier.vis.destroy_window()
    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
