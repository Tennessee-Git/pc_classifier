import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

LIDAR_INPUT_TOPIC = "/lidar/points"
CLASSIFIED_IMAGE = '/images'
TIMER_PERIOD = 0.05

class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier')
        self.subscription = self.create_subscription(
            Image, # do zmiany
            LIDAR_INPUT_TOPIC,
            self.classify_callback,
            10
        )
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(Image, CLASSIFIED_IMAGE, 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.lidar_publish_callback) # pewnie do wyjebania

    def classify_callback(self, msg):
        self.get_logger().info('CLASSIFIER RECEIVED MESSAGE')
        # klasyfikacja -> przeslanie dalej
        frame = self.br.imgmsg_to_cv2(msg)
        cv2.imshow("Unclassified image", frame)
        cv2.waitKey(1)

    def lidar_publish_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(
                self.br.cv2_to_imgmsg(frame, 'bgr8')
            )
            self.get_logger().info("Publishing frame")




def main(args=None):
    rclpy.init(args=args)
    classifier = ClassifierNode()
    rclpy.spin(classifier)

    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
