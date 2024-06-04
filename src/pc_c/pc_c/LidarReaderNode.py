import ctypes
from pc_c.Utils import convert_to_type
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from pc_c_interfaces.msg import LidarData, XyzPoint, RgbPoint
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import struct
from pc_c.Constants import LIDAR_TOPIC,LIDAR_DATA_TOPIC

class LidarReaderNode(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            PointCloud2,
            LIDAR_TOPIC,
            self.lidar_callback,
            10
        )
        self.publisher_ = self.create_publisher(LidarData, LIDAR_DATA_TOPIC, 10)

    def lidar_callback(self, msg):
        xyz = []
        gen = pc2.read_points(msg)
        int_data = list(gen)

        for x in int_data:
            xyz.append([x[0], x[1], x[2]])

        self.publish_data(np.array(xyz))

    def publish_data(self, xyz):
        msg = LidarData()

        msg.points = convert_to_type(xyz, XyzPoint, "p")

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReaderNode()
    rclpy.spin(lidar_reader)

    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
