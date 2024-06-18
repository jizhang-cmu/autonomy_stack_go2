#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

mpl.rcParams['toolbar'] = 'None'
plt.ion()

time_duration = 0
start_time_duration = 0
first_iteration = 'True'

explored_volume = 0
traveling_distance = 0
run_time = 0
max_explored_volume = 0
max_traveling_diatance = 0
max_run_time = 0

time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])
run_time_list = np.array([])
explored_volume_list = np.array([])
traveling_distance_list = np.array([])

def timeDurationCallback(msg):
    global time_duration, start_time_duration, first_iteration
    time_duration = msg.data
    if first_iteration == 'True':
        start_time_duration = time_duration
        first_iteration = 'False'

def runTimeCallback(msg):
    global run_time
    run_time = msg.data

def exploredVolumeCallback(msg):
    global explored_volume
    explored_volume = msg.data

def travelingDistanceCallback(msg):
    global traveling_distance
    traveling_distance = msg.data

class Listener(Node):

    def __init__(self):
        global time_duration, start_time_duration, explored_volume, traveling_distance, run_time, max_explored_volume, max_traveling_diatance, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_volume_list, traveling_distance_list
        super().__init__('realTimePlot')

        self.fig=plt.figure(figsize=(8,7))
        self.fig1=self.fig.add_subplot(311)
        plt.title("Exploration Metrics\n", fontsize=14)
        plt.margins(x=0.001)
        self.fig1.set_ylabel("Explored\nVolume (m$^3$)", fontsize=12)
        self.l1, = self.fig1.plot(time_list2, explored_volume_list, color='r', label='Explored Volume')
        self.fig2=self.fig.add_subplot(312)
        self.fig2.set_ylabel("Traveling\nDistance (m)", fontsize=12)
        self.l2, = self.fig2.plot(time_list3, traveling_distance_list, color='r', label='Traveling Distance')
        self.fig3=self.fig.add_subplot(313)
        self.fig3.set_ylabel("Algorithm\nRuntime (s)", fontsize=12)
        self.fig3.set_xlabel("Time Duration (s)", fontsize=12) #only set once
        self.l3, = self.fig3.plot(time_list1, run_time_list, color='r', label='Algorithm Runtime')

        self.fig.canvas.draw()

        self.count = 0

        self.time_duration_subscription = self.create_subscription(
            Float32,
            '/time_duration',
            timeDurationCallback,
            10)
        self.time_duration_subscription  # prevent unused variable warning 

        self.runtime_subscription = self.create_subscription(
            Float32,
            '/runtime',
            runTimeCallback,
            10)
        self.runtime_subscription  # prevent unused variable warning 

        self.explored_volume_subscription = self.create_subscription(
            Float32,
            '/explored_volume',
            exploredVolumeCallback,
            10)
        self.explored_volume_subscription  

        self.traveling_distance_subscription = self.create_subscription(
            Float32,
            '/traveling_distance',
            travelingDistanceCallback,
            10)
        self.traveling_distance_subscription  

        timer_period = 0.01  # 100hz 
        self.timer = self.create_timer(timer_period, self.plot_callback)

    def plot_callback(self):
        global time_duration, start_time_duration, explored_volume, traveling_distance, run_time, max_explored_volume, max_traveling_diatance, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_volume_list, traveling_distance_list
        self.count = self.count + 1

        if self.count % 25 == 0:
            max_explored_volume = explored_volume
            max_traveling_diatance = traveling_distance
            if run_time > max_run_time:
                max_run_time = run_time

            time_list2 = np.append(time_list2, time_duration)
            explored_volume_list = np.append(explored_volume_list, explored_volume)
            time_list3 = np.append(time_list3, time_duration)
            traveling_distance_list = np.append(traveling_distance_list, traveling_distance)
            time_list1 = np.append(time_list1, time_duration)
            run_time_list = np.append(run_time_list, run_time)

        if self.count >= 100:
            self.count = 0
            self.l1.set_xdata(time_list2)
            self.l2.set_xdata(time_list3)
            self.l3.set_xdata(time_list1)
            self.l1.set_ydata(explored_volume_list)
            self.l2.set_ydata(traveling_distance_list)
            self.l3.set_ydata(run_time_list)

            self.fig1.set_ylim(0, max_explored_volume + 500)
            self.fig1.set_xlim(start_time_duration, time_duration + 10)
            self.fig2.set_ylim(0, max_traveling_diatance + 20)
            self.fig2.set_xlim(start_time_duration, time_duration + 10)
            self.fig3.set_ylim(0, max_run_time + 0.2)
            self.fig3.set_xlim(start_time_duration, time_duration + 10)

            self.fig.canvas.draw()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
