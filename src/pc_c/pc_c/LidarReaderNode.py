import ctypes
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import struct
from PIL import Image as PImage
from cv_bridge import CvBridge

# LIDAR_TOPIC = "/ouster/points"
LIDAR_TOPIC = "/velodyne_points"
LIDAR_OUTPUT_TOPIC = "/lidar/points"
TIMER_PERIOD = 0.1

class LidarReaderNode(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            PointCloud2,
            LIDAR_TOPIC,
            self.lidar_callback,
            10
        )
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(any, LIDAR_OUTPUT_TOPIC, 10) # TODO: napisac custom type


    def lidar_callback(self, msg):
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        gen = pc2.read_points(msg)
        int_data = list(gen)

        for x in int_data:
            test = x[3]
            s = struct.pack('>f', test)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000)>>16
            g = (pack & 0x0000FF00)>>8
            b = (pack & 0x000000FF)

            xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
            rgb = np.append(rgb, [[r, g, b]], axis=0)

        self.get_logger().info(f'{msg.height, msg.width, xyz.shape}')
        self.convert_to_image(rgb.reshape((16, -1, 3)))


    def convert_to_image(self, rgb): #testowe
        print(rgb.shape)

        # img = PImage.fromarray(np.reshape(rgb, (height, width)))
        # self.publisher_.publish(
        #     self.br.cv2_to_imgmsg(img)
        # )



    def lidar_publish_callback(self):
        self.get_logger().info("PUBLISHING converted point clouds")

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReaderNode()
    rclpy.spin(lidar_reader)

    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
