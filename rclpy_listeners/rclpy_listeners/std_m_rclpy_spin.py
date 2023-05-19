import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import threading
   
    
class Subscriber(Node):
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


class RclpySpin(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, args=None):
        super(RclpySpin, self).__init__()
        self._stop_event = threading.Event()
        self._args = args

    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = Subscriber()

        rclpy.spin(self.subscriber)

    def stop(self):
        self._stop_event.set()

    def get_received_messages(self):
        return self.subscriber.received_messages

    def stopped(self):
        return self._stop_event.is_set()