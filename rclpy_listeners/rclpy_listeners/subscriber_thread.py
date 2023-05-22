import threading
import time
import psutil


class SubscriberThread(threading.Thread):

    def __init__(self, Sub, args=None):
        super(SubscriberThread, self).__init__()
        self._stop_event = threading.Event()
        self.Sub = Sub
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