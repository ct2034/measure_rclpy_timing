import rclpy

from rclpy_listeners.subscriber_thread import SubscriberThread, Subscriber


class RclpySpin(SubscriberThread):

    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = Subscriber(self.MessageType)

        rclpy.spin(self.subscriber)
