import rclpy
from rclpy.node import Node
from pc_c_interfaces.msg import LidarData
import open3d as o3d
import numpy as np
from pc_c.Utils import convert_to_array
from pc_c.Constants import LIDAR_DATA_TOPIC

class Display3DNode(Node):
    def __init__(self):
        super().__init__("display3d")
        self.subscription = self.create_subscription(
            LidarData,
            LIDAR_DATA_TOPIC,
            self.image_callback,
            10
        )
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.random.rand(1000,3))

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Display3D", width=1280, height=720)
        vis.add_geometry(pcd)
        ctr = vis.get_view_control()
        rop = vis.get_render_option()
        rop.light_on = False
        rop.point_size = 3.0

        self.pcd = pcd
        self.vis = vis
        self.ctr = ctr

    def image_callback(self, msg):
        xyz = convert_to_array(msg.points)
        self.get_logger().info(f'RECEIVED {xyz.shape}')
        self.visualize_lidar(xyz)

    def visualize_lidar(self, xyz):
        self.pcd.points = o3d.utility.Vector3dVector(xyz)
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        self.ctr.set_lookat(np.array([4,0,1]))
        self.ctr.set_zoom(3.5)

def main(args=None):
    rclpy.init(args=args)
    display = Display3DNode()
    rclpy.spin(display)

    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
