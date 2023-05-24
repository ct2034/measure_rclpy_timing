import itertools
import os
import random
import subprocess
import time

from matplotlib import pyplot as plt

import numpy as np

import pandas as pd

import rclpy

from std_msgs.msg import String

from custom_msgs.msg import NonStdString

from rclpy_listeners.ex_multithreaded import MultiThreadedEx
from rclpy_listeners.ex_rclpy_spin import RclpySpin
from rclpy_listeners.ex_rclpy_spin_once import RclpySpinOnce
from rclpy_listeners.subscriber_thread import Subscriber

import seaborn as sns


def run_listener(duration_s, MessageType, args=None):
    thread_listener = RclpySpin(MessageType, args=args)
    thread_listener.start()
    time.sleep(duration_s)
    n_messages_received = thread_listener.get_received_messages()
    print(f'drained {n_messages_received} messages')
    thread_listener.stop()
    rclpy.shutdown()
    thread_listener.join()
    return n_messages_received


def drain_queue(duration_s, MessageType, args=None):
    n_messages_received = run_listener(duration_s, MessageType, args)
    while n_messages_received:
        n_messages_received = run_listener(duration_s, MessageType, args)
    time.sleep(.1)


def make_measurement(duration_s, rate_hz, msg_size, Exec, MessageType, args=None):
    # preparation
    thread_listener = Exec(MessageType, args=args)
    PUBLISHER_RATIO = 2
    n_msgs_expected, process_pub = make_publisher(PUBLISHER_RATIO, duration_s, rate_hz, msg_size, MessageType)
    time.sleep(duration_s / 2)  # using 1/2 of the time to warm up
   
    # start the measurement
    thread_listener.start()

    # wait for the measurement to finish
    time.sleep(duration_s / 3)
    cpu_usage = thread_listener.get_cpu_usage()
    time.sleep(duration_s / 3)
    cpu_usage = float(np.mean([
        cpu_usage,
        thread_listener.get_cpu_usage()]))
    time.sleep(duration_s / 3)

    # stop the measurement
    thread_listener.stop()
    n_messages_received = thread_listener.get_received_messages()
    process_pub.kill()

    # report results
    print(f'{n_messages_received=}')
    print(f'{n_messages_received/n_msgs_expected=}')
    print(f'{cpu_usage=}')

    # clean up
    rclpy.shutdown()
    thread_listener.join()
    time.sleep((PUBLISHER_RATIO - 1) * duration_s)
    print('thread finished...exiting')
    return n_messages_received/n_msgs_expected, cpu_usage


def make_publisher(PUBLISHER_RATIO, duration_s, rate_hz, msg_size, MessageType):
    n_msgs_expected = duration_s * rate_hz
    # PUBLISHER_RATIO x in case of loss
    n_msgs_send = PUBLISHER_RATIO * n_msgs_expected
    if MessageType == String:
        msg_type = 'std_msgs/msg/String'
    elif MessageType == NonStdString:
        msg_type = 'custom_msgs/msg/NonStdString'
    else:
        raise NotImplementedError
    data_content = 'x' * msg_size
    data_str = f'data: {data_content}'
    content_str = '{' + data_str + '}'
    pub_command = (
        f'ros2 topic pub -t {n_msgs_send} -w 0 -r {rate_hz} -p 0 ' 
        f'/topic {msg_type} \'{content_str}\'')
    info = (pub_command[:100] + '..') if len(pub_command) > 75 else pub_command
    print(f'pub_command={info}')
    process_pub = subprocess.Popen(
        f'bash -c "source /opt/ros/{os.environ["ROS_DISTRO"]}/setup.bash; '
        f'{pub_command}"',
        shell=True)
        
    return n_msgs_expected, process_pub


def experiment(args=None):
    # general params
    duration = 2
    drain_duration = duration/2
    df = pd.DataFrame(columns=[
        'frequency', 'msg_size', 'trial', 'executor', 
        'message_type', 'delivery_ratio', 'cpu_usage'])

    # experiment params
    frequencies = np.logspace(start=1, stop=4, num=4, dtype=int)
    # frequencies = np.logspace(start=1, stop=3, num=2, dtype=int)
    n_trials = 3
    # n_trials = 1
    msg_sizes = np.logspace(start=0, stop=4, num=3, dtype=int)
    # msg_sizes = np.logspace(start=0, stop=4, num=2, dtype=int)
    executors = [RclpySpin, RclpySpinOnce, MultiThreadedEx]
    message_types = [String, NonStdString]

    # experiment indexing
    experiments = dict(
        enumerate(itertools.product(
            frequencies, 
            msg_sizes,
            range(n_trials), 
            executors, 
            message_types
        ))
    )
    order = list(experiments.keys())
    random.shuffle(order)

    # experiment
    start_overall = time.time()
    for t, (frequency, msg_size, trial, Exec, MessageType
            ) in enumerate(experiments[i] for i in order):
        print(f'{t=} / {len(experiments)}')
        delivery_ratio, cpu_usage = make_measurement(
            duration, frequency, msg_size, Exec, MessageType, args=args)
        drain_queue(drain_duration, MessageType, args=args)
        drain_queue(drain_duration, MessageType, args=args)
        df = pd.concat([df, pd.DataFrame({
            'frequency': [frequency],
            'msg_size': [msg_size],
            'trial': [trial],
            'executor': [Exec.__name__],
            'message_type': [MessageType.__name__],
            'delivery_ratio': [delivery_ratio],
            'cpu_usage': [cpu_usage]})], ignore_index=True)

    # save results
    df.to_csv('data.csv')
    print(f'overall runtime: {(time.time() - start_overall) / 60} min')


def plot():
    df = pd.read_csv('data.csv')
    _pairplot(df, 'message_type')
    _pairplot(df, 'executor')


def _pairplot(df, hue):
    pp = sns.pairplot(
        df,
        x_vars=['frequency', 'msg_size'],
        y_vars=['delivery_ratio', 'cpu_usage'],
        hue=hue,
    )
    for ax in pp.axes.flat:
        ax.set_xscale('log')
    pp.map_offdiag(sns.lineplot)
    plt.savefig(f'pairplot_{hue}.png')
    plt.close()


def main(args=None):
    experiment(args=args)
    plot()


if __name__ == '__main__':
    main()