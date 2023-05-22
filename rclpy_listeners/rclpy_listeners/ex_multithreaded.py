import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy_listeners.subscriber_thread import SubscriberThread, Subscriber


class MultiThreadedEx(SubscriberThread):

    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = Subscriber(self.MessageType)

        executor = MultiThreadedExecutor()
        executor.add_node(self.subscriber)
        executor.spin()
