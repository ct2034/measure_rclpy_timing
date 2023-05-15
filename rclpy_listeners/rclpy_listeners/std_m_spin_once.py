import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received_messages = 0

    def listener_callback(self, _):
        self.received_messages += 1
        print(f'{self.received_messages=}')
        # if self.received_messages >= 5:
        #     self.destroy_node()


def main(args=None, ros_context=None):
    rclpy.init(args=args, context=ros_context)

    minimal_subscriber = MinimalSubscriber()

    while minimal_subscriber.received_messages < 5 and rclpy.ok():
        rclpy.spin_once(minimal_subscriber)
    print(f'{minimal_subscriber.received_messages=}')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
