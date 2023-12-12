#!/usr/bin/env python3

# Columbia Engineering
# MECS 4603 - Fall 2023

import math
import numpy
import time

import rclpy
from rclpy.node import Node

from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import SensorData

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')

        # Publisher to publish state estimate
        self.pub_est = self.create_publisher(RobotPose, "/robot_pose_estimate", 1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.05

        # Subscribe to command input and sensory output of robot
        self.sensor_sub = self.create_subscription(SensorData, "/sensor_data", self.sensor_callback, 1)
        
    
    def estimate(self, sens: SensorData):
        '''This function gets called every time the robot publishes its control 
        input and sensory output. You must make use of what you know about 
        extended Kalman filters to come up with an estimate of the current
        state of the robot and covariance matrix. The SensorData message 
        contains fields 'vel_trans' and 'vel_ang' for the commanded 
        translational and rotational velocity respectively. Furthermore, 
        it contains a list 'readings' of the landmarks the robot can currently
        observe

        Args:
            sens: incoming sensor message
        '''
        #TODO: implement your extended Kalman filter here
        # Prediction Step
        vel_trans = sens.vel_trans
        vel_ang = sens.vel_ang
        theta = self.x[2, 0]
        self.x[0] += self.step_size * vel_trans * math.cos(theta)
        self.x[1] += self.step_size * vel_trans * math.sin(theta)
        self.x[2] += self.step_size * vel_ang
        F = numpy.eye(3)
        F[0, 2] = -self.step_size * vel_trans * math.sin(theta)
        F[1, 2] = self.step_size * vel_trans * math.cos(theta)
        self.P = F @ self.P @ F.T + self.V
        # Update Step
        for reading in sens.readings:
            xl, yl = reading.landmark.x, reading.landmark.y
            z_range = reading.range
            z_bearing = reading.bearing
            dx = xl - self.x[0]
            dy = yl - self.x[1]
            predicted_range = math.sqrt(dx**2 + dy**2)
            predicted_bearing = math.atan2(dy, dx) - self.x[2]
            predicted_bearing = (predicted_bearing + math.pi) % (2 * math.pi) - math.pi
            H = numpy.zeros((2, 3))
            H[0, 0] = -dx / predicted_range
            H[0, 1] = -dy / predicted_range
            H[1, 0] = dy / (predicted_range**2)
            H[1, 1] = -dx / (predicted_range**2)
            H[1, 2] = -1
            R = numpy.array([[0.1, 0], [0, 0.05]])
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ numpy.linalg.inv(S)
            y_bearing = z_bearing - predicted_bearing
            y_bearing = (y_bearing+ math.pi) % (2 * math.pi) - math.pi
            y = numpy.array([[z_range - predicted_range], [y_bearing]])
            self.x = self.x.astype(numpy.float64)
            K = K.astype(numpy.float64)
            y = y.astype(numpy.float64)
            self.x += K @ y
            self.P = (numpy.eye(3) - K @ H) @ self.P



    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = float(self.x[0])
        est_msg.pose.y = float(self.x[1])
        est_msg.pose.theta = float(self.x[2])
        self.pub_est.publish(est_msg)

def main(args=None):
    rclpy.init(args=args)   
    est = Estimator()
    rclpy.spin(est)
                
if __name__ == '__main__':
   main()

 
