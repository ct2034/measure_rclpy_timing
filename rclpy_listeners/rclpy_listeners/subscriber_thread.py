import threading
import time
import psutil

from rclpy.node import Node


class SubscriberThread(threading.Thread):

    def __init__(self, MessageType, args=None):
        super(SubscriberThread, self).__init__()
        self._stop_event = threading.Event()
        self.MessageType = MessageType
        self._args = args
        self.subscriber = None

    def run(self):
        raise NotImplementedError

    def stop(self):
        self._stop_event.set()

    def get_received_messages(self):
        while not self.subscriber:
            time.sleep(.1)
        return self.subscriber.received_messages

    def get_cpu_usage(self):
        return psutil.Process().cpu_percent(.1)

    def stopped(self):
        return self._stop_event.is_set()
    

class Subscriber(Node):

    def __init__(self, MessageType):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            MessageType,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received_messages = 0

    def listener_callback(self, _):
        self.received_messages += 1
        # print(f'{self.received_messages=}')
