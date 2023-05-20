import itertools
import os
import subprocess
import time

from matplotlib import pyplot as plt

import numpy as np

import pandas as pd

import rclpy

from rclpy_listeners.std_m_multithreaded_ex import MultiThreadedEx
from rclpy_listeners.std_m_rclpy_spin import RclpySpin
from rclpy_listeners.std_m_rclpy_spin_once import RclpySpinOnce

import seaborn as sns


def run_listener(duration_s, args=None):
    thread_listener = RclpySpin(args=args)
    thread_listener.start()
    time.sleep(duration_s)
    n_messages_received = thread_listener.get_received_messages()
    print(f'drained {n_messages_received} messages')
    thread_listener.stop()
    rclpy.shutdown()
    thread_listener.join()
    return n_messages_received


def drain_queue(duration_s, args=None):
    n_messages_received = run_listener(duration_s, args)
    while n_messages_received:
        n_messages_received = run_listener(duration_s, args)
    time.sleep(.1)


def make_measurement(duration_s, rate_hz, Sub, args=None):
    PUBLISHER_RATIO = 2
    n_msgs_expected = duration_s * rate_hz
    # PUBLISHER_RATIO x in case of loss
    n_msgs_send = PUBLISHER_RATIO * n_msgs_expected
    pub_command = f'ros2 topic pub -t {n_msgs_send} -w 0 -r {rate_hz}' + \
        ' -p 0 /topic std_msgs/msg/String'
    print(f'{pub_command=}')
    process_pub = subprocess.Popen(
        f'bash -c "source /opt/ros/{os.environ["ROS_DISTRO"]}/setup.bash; {pub_command}"',
        shell=True)
    time.sleep(.1)
    thread_listener = Sub(args=args)
    thread_listener.start()
    time.sleep(duration_s / 3)
    cpu_usage = thread_listener.get_cpu_usage()
    time.sleep(duration_s / 3)
    cpu_usage = float(np.mean([
        cpu_usage,
        thread_listener.get_cpu_usage()]))
    time.sleep(duration_s / 3)
    n_messages_received = thread_listener.get_received_messages()
    print(f'{n_messages_received=}')
    print(f'{n_messages_received/n_msgs_expected=}')
    print(f'{cpu_usage=}')
    thread_listener.stop()
    rclpy.shutdown()
    process_pub.kill()
    thread_listener.join()
    time.sleep((PUBLISHER_RATIO - 1) * duration_s)
    print('thread finished...exiting')
    return n_messages_received/n_msgs_expected, cpu_usage


def experiment(args=None):
    subscribers = [RclpySpin, RclpySpinOnce, MultiThreadedEx]
    frequencies = np.logspace(start=0, stop=6, num=7, dtype=int)
    duration = 5
    drain_duration = 1
    n_trials = 3
    # n_trials = 1

    df = pd.DataFrame(columns=[
        'frequency', 'trial', 'subscriber', 'delivery_ratio', 'cpu_usage'])

    for i, (frequency, trial, Sub) in enumerate(itertools.product(
            frequencies, range(n_trials), subscribers)):
        print(f'{i=} / {len(frequencies) * n_trials * len(subscribers)}')
        drain_queue(drain_duration, args=args)
        delivery_ratio, cpu_usage = make_measurement(
            duration, frequency, Sub, args=args)
        drain_queue(drain_duration, args=args)
        df = pd.concat([df, pd.DataFrame({
            'frequency': [frequency],
            'trial': [trial],
            'subscriber': [Sub.__name__],
            'delivery_ratio': [delivery_ratio],
            'cpu_usage': [cpu_usage]})], ignore_index=True)

    df.to_csv('data.csv')


def plot():
    df = pd.read_csv('data.csv')
    _lineplot(df, 'delivery_ratio')
    plt.figure()
    _lineplot(df, 'cpu_usage')


def _lineplot(df, y):
    sns.lineplot(data=df, x='frequency', y=y, hue='subscriber')
    plt.xscale('log')
    plt.savefig(f'{y}.png')


def main(args=None):
    experiment(args=args)
    plot()
