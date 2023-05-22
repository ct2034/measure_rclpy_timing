

from rclpy.node import Node

from std_msgs.msg import String


class StdMsgSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received_messages = 0

    def listener_callback(self, _):
        self.received_messages += 1
        # print(f'{self.received_messages=}')
