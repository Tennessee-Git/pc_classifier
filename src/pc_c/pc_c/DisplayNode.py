import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

    def image_callback(self, msg):
        height = msg.height
        width = msg.width
        frame = self.br.imgmsg_to_cv2(msg.data)
        self.get_logger().info("Image received. H: %s W: %s" % height, width)

def main(args=None):
    rclpy.init(args=args)
    display = DisplayNode()
    rclpy.spin(display)

    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
