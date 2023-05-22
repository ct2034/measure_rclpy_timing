import rclpy

from rclpy_listeners.subscriber_thread import SubscriberThread


class RclpySpin(SubscriberThread):

    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = self.Sub()

        rclpy.spin(self.subscriber)