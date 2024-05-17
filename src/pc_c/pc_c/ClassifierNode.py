import rclpy
from rclpy.node import Node

LIDAR_INPUT_TOPIC = "/lidar"
CLASSIFIED_IMAGE = '/images'
TIMER_PERIOD = 0.1

class Classifier(Node):
    def __init__(self):
        super().__init__('classifier')
        self.subscription = self.create_subscription(
            any,
            LIDAR_INPUT_TOPIC,
            self.classify_callback,
            10
        )
        self.publisher_ = self.create_publisher(any, CLASSIFIED_IMAGE, 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.lidar_publish_callback)

    def classify_callback(self, msg):
        self.get_logger().info('CLASSIFIER RECEIVED: %s' % msg.data)
        # klasyfikacja

def main(args=None):
    rclpy.init(args=args)
    classifier = Classifier()
    rclpy.spin(classifier)

    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
