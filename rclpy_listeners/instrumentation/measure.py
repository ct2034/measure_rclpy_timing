import itertools

from rclpy_listeners.std_m_rclpy_spin import RclpySpin
from rclpy_listeners.std_m_rclpy_spin_once import RclpySpinOnce

import rclpy
import time
from matplotlib import pyplot as plt

import pandas as pd
import seaborn as sns

import subprocess


def drain_queue(duration_s, args=None):
    thread_listener = RclpySpin(args=args)
    thread_listener.start()
    time.sleep(duration_s)
    n_messages_received = thread_listener.get_received_messages()
    print(f"drained {n_messages_received} messages")
    thread_listener.stop()
    rclpy.shutdown()
    thread_listener.join()
    time.sleep(duration_s)


def make_measurement(duration_s, rate_hz, Sub, args=None):
    PUBLISHER_RATIO = 2
    n_msgs_expected = duration_s * rate_hz  # what we expect to receive
    n_msgs_send = PUBLISHER_RATIO * n_msgs_expected  # PUBLISHER_RATIO x in case of loss
    print_period = 0
    pub_command = f'ros2 topic pub --times {n_msgs_send} -r {rate_hz} -p {print_period} /topic std_msgs/msg/String'
    print(f"{pub_command=}")
    process_pub = subprocess.Popen(
        f'bash -c "source /opt/ros/rolling/setup.bash; {pub_command}"',
        shell=True)
    thread_listener = Sub(args=args)
    thread_listener.start()
    # thread_publisher.start()
    # thread_publisher.join()
    time.sleep(duration_s)
    n_messages_received = thread_listener.get_received_messages()
    print(f"{n_messages_received=}")
    print(f"{n_messages_received/n_msgs_expected=}")
    thread_listener.stop()
    rclpy.shutdown()
    process_pub.kill()
    thread_listener.join()
    time.sleep((PUBLISHER_RATIO - 1) * duration_s)
    print("thread finished...exiting")
    return n_messages_received/n_msgs_expected


def experiment(args=None):
    subscribers = [RclpySpin, RclpySpinOnce]
    frequencies = [1, 3, 10, 30, 100, 300, 1000, 3000, 10000]
    duration = 3
    drain_duration = 1
    n_trials = 3

    df = pd.DataFrame(columns=["frequency", "trial", "delivery_ratio", "subscriber"])

    for frequency, trial, Sub in itertools.product(
            frequencies, range(n_trials), subscribers):
        drain_queue(drain_duration, args=args)
        delivery_ratio = make_measurement(duration, frequency, Sub, args=args)
        drain_queue(drain_duration, args=args)
        df = df.append({
            "frequency": frequency, 
            "trial": trial, 
            "delivery_ratio": delivery_ratio,
            "subscriber": Sub.__name__
        }, ignore_index=True)

    df.to_csv("delivery_ratio.csv")


def plot():
    df = pd.read_csv("delivery_ratio.csv")
    sns.lineplot(data=df, x="frequency", y="delivery_ratio", hue="subscriber")
    plt.xscale("log")
    plt.savefig("delivery_ratio.png")


def main(args=None):
    experiment(args=args)
    plot()