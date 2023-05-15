import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, frequency, number, length):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1 / frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.n = number
        self.length = length
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'x' * self.length
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        if self.i == self.n:
            self.destroy_node()
            rclpy.shutdown()


def main(args, frequency, number, length, ros_context=None):
    rclpy.init(args=args, context=ros_context)

    minimal_publisher = MinimalPublisher(frequency, number, length)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
