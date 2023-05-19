import rclpy

from rclpy_listeners.std_m_base import SubscriberThread, Subscriber
    

class RclpySpin(SubscriberThread):
    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = Subscriber()

        rclpy.spin(self.subscriber)