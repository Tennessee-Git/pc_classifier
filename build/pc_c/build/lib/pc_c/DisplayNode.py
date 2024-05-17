import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

IMAGE_INPUT_TOPIC = '/images'
TIMER_PERIOD = 0.1

class DisplayNode(Node):
    def __init__(self):
        super().__init__("display")
        self.subscription = self.create_subscription(
            Image,
            IMAGE_INPUT_TOPIC,
            self.image_callback,
            10
        )
        self.br = CvBridge()

    def image_callback(self, data):
        self.get_logger().info("Image received.")
        frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("Classifed image", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    display = DisplayNode()
    rclpy.spin(display)

    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
