from threading import Thread

from rclpy_listeners.std_m_spin_once import main as std_m_spin_once_main
from rclpy_talker.publisher import main as publisher_main

import rclpy

import subprocess

def main(args=None):
    thread_listener = Thread(
        target=std_m_spin_once_main, 
        args=(args,))
    # thread_publisher = Thread(
    #     target=publisher_main, 
    #     args=(args, 10, 10, 5, context))
    thread_listener.start()
    times = 10
    rate = 100
    print_period = 0
    subprocess.run(
        'bash -c "source /opt/ros/rolling/setup.bash; ' +
        f'ros2 topic pub --times {times} -r {rate} -p {print_period} /topic std_msgs/msg/String"',
        shell=True)
    # thread_publisher.start()
    # thread_publisher.join()
    thread_listener.join()
    print("thread finished...exiting")