import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

LIDAR_INPUT_TOPIC = "/lidar"
CLASSIFIED_IMAGE = '/images'
TIMER_PERIOD = 0.1

class Classifier(Node):
    def __init__(self):
        super().__init__('classifier')
        self.subscription = self.create_subscription(
            String,
            LIDAR_INPUT_TOPIC,
            self.classify_callback,
            10
        )
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(Image, CLASSIFIED_IMAGE, 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.lidar_publish_callback)

    def classify_callback(self, data):
        self.get_logger().info('CLASSIFIER RECEIVED: %s' % data)
        # klasyfikacja

    def lidar_publish_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(
                self.br.cv2_to_imgmsg(frame, 'bgr8')
            )
            self.get_logger().info("Publishing frame")




def main(args=None):
    rclpy.init(args=args)
    classifier = Classifier()
    rclpy.spin(classifier)

    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
