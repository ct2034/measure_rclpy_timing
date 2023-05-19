import rclpy


from rclpy_listeners.std_m_base import SubscriberThread, Subscriber
    

class RclpySpinOnce(SubscriberThread):
    def run(self):
        rclpy.init(args=self._args, context=None)

        self.subscriber = Subscriber()

        while rclpy.ok():
            rclpy.spin_once(self.subscriber)

    