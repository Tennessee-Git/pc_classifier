import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pc_c_interfaces.msg import LidarData, XyzPoint, RgbPoint
import cv2
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
        self.get_logger().info('CLASSIFIER RECEIVED MESSAGE')
        # klasyfikacja -> przeslanie dalej
        xyz = np.array(msg.points)
        rgb = np.array(msg.rgb_points)
        print(xyz.shape,"\n",xyz[:2],'\n','\n', rgb.shape,rgb[:2],"\n")

    def lidar_publish_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(
                self.br.cv2_to_imgmsg(frame, 'bgr8')
            )
            # self.get_logger().info("Publishing frame")




def main(args=None):
    rclpy.init(args=args)
    classifier = ClassifierNode()
    rclpy.spin(classifier)

    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
