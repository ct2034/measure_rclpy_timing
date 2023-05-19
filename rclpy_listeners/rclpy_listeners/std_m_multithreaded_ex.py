import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy_listeners.std_m_base import Subscriber, SubscriberThread


class MultiThreadedEx(SubscriberThread):

    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = Subscriber()
        executor = MultiThreadedExecutor()
        executor.add_node(self.subscriber)
        executor.spin()
