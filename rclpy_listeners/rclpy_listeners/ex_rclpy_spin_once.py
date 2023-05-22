import rclpy

from rclpy_listeners.subscriber_thread import SubscriberThread


class RclpySpinOnce(SubscriberThread):

    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = self.Sub()

        while rclpy.ok():
            rclpy.spin_once(self.subscriber)
