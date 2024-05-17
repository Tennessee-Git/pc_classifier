import rclpy
from rclpy.node import Node

LIDAR_TOPIC = "/ouster/points"
LIDAR_OUTPUT_TOPIC = "/lidar"
TIMER_PERIOD = 0.1

class LidarReaderNode(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            any, # znalezc wlasciwy typ
            LIDAR_TOPIC,
            self.lidar_callback,
            10
        )
        # self.publisher_ = self.create_publisher(any, LIDAR_OUTPUT_TOPIC, 10)
        # self.timer = self.create_timer(TIMER_PERIOD, self.lidar_publish_callback)


    def lidar_callback(self, msg):
        self.get_logger().info('RECEIVED: %s OF TYPE:' % msg.data, type(msg.data))

    def lidar_publish_callback(self):
        print()
        self.publisher_.publish()
        self.get_logger().info("PUBLISHING: %s")

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReaderNode()
    rclpy.spin(lidar_reader)

    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
