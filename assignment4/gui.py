#!/usr/bin/env python3

# Columbia Engineering
# MECS 4603 - Fall 2023

import math
import numpy
import time
import threading

import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import Landmark
from state_estimator_msgs.msg import LandmarkSet
from state_estimator_msgs.msg import LandmarkReading
from state_estimator_msgs.msg import SensorData

def make_circle(r, x_offset, y_offset):
    t = numpy.arange(0, numpy.pi * 2.0, 0.01)
    t = t.reshape((len(t), 1))
    x = r * numpy.cos(t) + x_offset
    y = r * numpy.sin(t) + y_offset
    return numpy.hstack((x, y))

class GUI(Node):

    def __init__(self):
        super().__init__('gui')

        plt.ion()
        self._fig = plt.figure(figsize=(10,10))
        self._ax1 = self._fig.add_subplot(1,1,1)
        plt.show()

        self.lock = threading.Lock()

        self.cmap = cm.get_cmap("RdYlGn")
        self.color = 0.75
        self.base_color = 0.8
        self.decay = 0.4
        self.last_t = time.time()
        
        self.robot_pose = Pose2D()
        self.robot_pose_est = Pose2D()
        self.landmarks = []
        self.readings = []

        self.pose_sub = self.create_subscription(RobotPose, "/robot_pose", self.pose_callback, 1)
        self.est_sub = self.create_subscription(RobotPose, "/robot_pose_estimate", self.est_callback, 1)
        self.land_sub = self.create_subscription(LandmarkSet, "/landmarks", self.landmarks_callback, 1)
        self.sensor_sub = self.create_subscription(SensorData, "/sensor_data", self.sensor_callback, 1)
        self.error_sub = self.create_subscription(Int8, "/student_error", self.error_callback, 1)

        self.odom_error = None
        self.est_error = None
        self.error = 0

        self.aoe_sub = self.create_subscription(Float64, "/acceptable_odometry_error", self.odometry_error_callback, 1)
        self.aee_sub = self.create_subscription(Float64, "/acceptable_estimation_error", self.estimation_error_callback, 1)

    def odometry_error_callback(self, msg):
        self.odom_error = msg.data

    def estimation_error_callback(self, msg):
        self.est_error = msg.data
    
    def sensor_callback(self, msg):
        self.readings = msg.readings
    
    def pose_callback(self, msg):
        self.robot_pose = msg.pose

    def est_callback(self, msg):
        self.robot_pose_est = msg.pose

    def landmarks_callback(self, msg):
        self.landmarks = msg.landmarks

    def error_callback(self, msg):
        self.lock.acquire()
        if msg.data == 1: self.color = 0
        if msg.data == 2 and self.color > 0.25: self.color = 0.25
        self.last_t = time.time()
        self.lock.release()

    def update(self):
        self._ax1.clear()
        plt.xlim(-10,10)
        plt.ylim(-10,10)
        ptx=[self.robot_pose.x]
        pty=[self.robot_pose.y]
        self._ax1.scatter(ptx, pty, c='r', s = 64);
        ptx=[self.robot_pose.x+0.2*math.cos(self.robot_pose.theta)]
        pty=[self.robot_pose.y+0.2*math.sin(self.robot_pose.theta)]
        self._ax1.scatter(ptx, pty, c='r', s = 16);
        
        ptx=[self.robot_pose_est.x]
        pty=[self.robot_pose_est.y]
        self._ax1.scatter(ptx, pty, c='b', s = 64);
        ptx=[self.robot_pose_est.x+0.2*math.cos(self.robot_pose_est.theta)]
        pty=[self.robot_pose_est.y+0.2*math.sin(self.robot_pose_est.theta)]
        self._ax1.scatter(ptx, pty, c='b', s = 16);

        range_circle = make_circle(2.5, self.robot_pose.x, self.robot_pose.y)
        range_circle_path = mpath.Path(range_circle)
        range_patch = mpatches.PathPatch(range_circle_path, alpha=0.05, facecolor='k', edgecolor='k')
        self._ax1.add_patch(range_patch)

        if self.odom_error:
            odom_circle = make_circle(self.odom_error, self.robot_pose.x, self.robot_pose.y)
            odom_circle_path = mpath.Path(odom_circle)
            odom_patch = mpatches.PathPatch(odom_circle_path, alpha=0.2, facecolor='y', edgecolor='y')
            self._ax1.add_patch(odom_patch)

        if self.est_error:
            est_circle = make_circle(self.est_error, self.robot_pose.x, self.robot_pose.y)
            est_circle_path = mpath.Path(est_circle)
            est_patch = mpatches.PathPatch(est_circle_path, alpha=0.2, facecolor='g', edgecolor='g')
            self._ax1.add_patch(est_patch)

        if self.odom_error or self.est_error:
            self.lock.acquire()
            self.color += self.decay * (time.time() - self.last_t)
            self.last_t = time.time()
            if self.color > self.base_color: self.color = self.base_color
            self._fig.patch.set_facecolor(self.cmap(self.color))
            self.lock.release()
    
        ptx = []
        pty = []
        for i in range(0,len(self.landmarks)):
            ptx.append(self.landmarks[i].x)
            pty.append(self.landmarks[i].y)
        self._ax1.scatter(ptx, pty, c='b', marker='*', s = 64);
        ptx = []
        pty = []
        for i in range(0,len(self.readings)):
            ptx.append(self.readings[i].landmark.x)
            pty.append(self.readings[i].landmark.y)
        self._ax1.scatter(ptx, pty, c='r', marker='*', s = 64);

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

def my_thread_func():
    rclpy.spin(gui)

def main(args=None):
    rclpy.init(args=args)
    global gui
    gui = GUI()

    # Need to spin in separate thread because GUI has to run in main thread
    my_thread = threading.Thread(target=my_thread_func)
    my_thread.daemon = True
    my_thread.start()
 
    while rclpy.ok():
        gui.update()

if __name__ == '__main__':
    main()
