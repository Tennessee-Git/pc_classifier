import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pc_c_interfaces.msg import LidarData
import cv2
import open3d as o3d
from cv_bridge import CvBridge
import numpy as np

LIDAR_INPUT_TOPIC = "/lidar/points"
CLASSIFIED_IMAGE = '/images'
TIMER_PERIOD = 0.05

class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier')
        self.subscription = self.create_subscription(
            LidarData,
            LIDAR_INPUT_TOPIC,
            self.classify_callback,
            10
        )
        self.br = CvBridge()
        self.lidar_height = 16
        self.lidar_width = 1824

        self.publisher_ = self.create_publisher(Image, CLASSIFIED_IMAGE, 10)

    def classify_callback(self, msg):
        xyz = self.convert_to_array(msg.points)
        rgb = self.convert_to_array(msg.rgb_points, True)
        self.get_logger().info(f'RECEIVED {xyz.shape, rgb.shape}')
        # self.visualize_lidar(xyz)

    def convert_to_array(self, arr, reshape=False):
        output = []
        arr = np.array(arr)
        for i in arr:
            output.append(i.p)

        extra_len = (self.lidar_height * self.lidar_width) - len(output)
        for j in range(extra_len):
            output.append([0,0,0])

        if reshape:
            return np.array(output).reshape(self.lidar_height, self.lidar_width, 3)
        else:
            return np.array(output)

    def visualize_lidar(self, arr):
        print("VISUALIZING")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(arr)
        pcd_center = pcd.get_center()
        pcd.translate(-pcd_center)
        o3d.visualization.draw_geometries([pcd])


def main(args=None):
    rclpy.init(args=args)
    classifier = ClassifierNode()
    rclpy.spin(classifier)

    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
