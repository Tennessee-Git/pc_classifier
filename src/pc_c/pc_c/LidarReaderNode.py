import ctypes
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
        rgb = []
        xyz = []
        gen = pc2.read_points(msg)
        int_data = list(gen)

        for x in int_data:
            # test = x[3]
            # s = struct.pack('>f', test)
            # i = struct.unpack('>l', s)[0]
            # pack = ctypes.c_uint32(i).value

            # r = (pack & 0x00FF0000)>>16
            # g = (pack & 0x0000FF00)>>8
            # b = (pack & 0x000000FF)

            xyz.append([x[0], x[1], x[2]])
            # rgb.append([r, g, b])

        self.publish_data(np.array(xyz), np.array(rgb))

    # def convert_to_XyzPoint(self, xyz):
    #     xyz = np.nan_to_num(xyz)
    #     xyz_points = []
    #     for i in xyz:
    #         coords = XyzPoint()
    #         coords.xp = i
    #         xyz_points.append(coords)
    #     return xyz_points

    # def convert_to_RgbPoint(self, rgb):
    #     rgb = np.nan_to_num(rgb)
    #     rgb = rgb.astype(np.uint8)
    #     rgb_points = []
    #     for i in rgb:
    #         rgb_point = RgbPoint()
    #         rgb_point.rp = i
    #         rgb_points.append(rgb_point)
    #     return rgb_points

    def convert_to_type(self, arr, T,p, is_rgb=False):
        arr = np.nan_to_num(arr)
        output = []
        if is_rgb:
            arr = arr.astype(np.uint8)
        for i in arr:
            el = T()
            # el.p = i
            setattr(el, p, i)
            output.append(el)
        return output

    def publish_data(self, xyz, rgb):
        msg = LidarData()
        # msg.points = self.convert_to_XyzPoint(xyz)
        # msg.rgb_points = self.convert_to_RgbPoint(rgb)

        msg.points = self.convert_to_type(xyz, XyzPoint, "p")
        # msg.rgb_points = self.convert_to_type(rgb, RgbPoint, "p", True)

        self.publisher_.publish(msg)
        # self.get_logger().info(f"PUBLISHING xyz:{len(xyz)} rgb:{len(rgb)}")
        # self.get_logger().info(f"PUBLISHING xyz:{len(xyz)}")

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReaderNode()
    rclpy.spin(lidar_reader)

    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
