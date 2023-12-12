#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import transforms3d
from pyquaternion import Quaternion
import math
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

	
class SolutionPublisher(Node):

    def __init__(self):
        super().__init__('solution')

        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):

        object = TransformStamped()
        robot = TransformStamped()
        camera = TransformStamped()

        object.header.stamp = self.get_clock().now().to_msg()
        object.header.frame_id = 'base_frame'
        object.child_frame_id = 'object_frame'

        quat_1 = transforms3d.euler.euler2quat(0.64, 0.64, 0, 'rxyz')
   	 
        object.transform.rotation.x = quat_1[1]
        object.transform.rotation.y = quat_1[2]
        object.transform.rotation.z = quat_1[3]
        object.transform.rotation.w = quat_1[0]

        R_1 = transforms3d.euler.euler2mat(0.64, 0.64, 0, 'rxyz')
        R_0 = transforms3d.euler.euler2mat(0, 0, 0, 'sxyz')
        T_r =  transforms3d.affines.compose([0,0,0], R_1, [1,1,1])
        T_t =  transforms3d.affines.compose([1.5, 0.8, 0], R_0, [1,1,1])
        trans_1 = np.dot(T_r, T_t)
        
        object.transform.translation.x = trans_1[0,3]
        object.transform.translation.y = trans_1[1,3]
        object.transform.translation.z = trans_1[2,3]

        robot.header.stamp = self.get_clock().now().to_msg()
        robot.header.frame_id = 'base_frame'
        robot.child_frame_id = 'robot_frame'

        quat_2 = transforms3d.euler.euler2quat(0, 1.5, 0, 'sxyz')

        robot.transform.rotation.x = quat_2[1]
        robot.transform.rotation.y = quat_2[2]
        robot.transform.rotation.z = quat_2[3]
        robot.transform.rotation.w = quat_2[0]
   	 
        R_2 = transforms3d.euler.euler2mat(0, 1.5, 0, 'sxyz')
        T_r2 =  transforms3d.affines.compose([0,0,0], R_2, [1,1,1])
        T_t2 =  transforms3d.affines.compose([0, 0, -2.0], R_0, [1,1,1])
        trans_2 = np.dot(T_r2, T_t2)
        robot.transform.translation.x = trans_2[0,3]
        robot.transform.translation.y = trans_2[1,3]
        robot.transform.translation.z = trans_2[2,3]

    
        camera.header.stamp = self.get_clock().now().to_msg()
        camera.header.frame_id = 'robot_frame'
        camera.child_frame_id = 'camera_frame'

        camera.transform.translation.x = 0.3
        camera.transform.translation.y = 0.0
        camera.transform.translation.z = 0.3
        
        transform_base_object = trans_1
        transform_base_robot = trans_2
        B =  transforms3d.affines.compose([0.3, 0.0, 0.3], R_0, [1,1,1])
        transform_base_camera = np.dot(trans_2,B)
        transform_camera_object = np.dot(np.linalg.inv(transform_base_camera),trans_1)
        translation, rotation, scale, shear = transforms3d.affines.decompose(transform_camera_object)
        vector = translation/np.linalg.norm(translation)
        X = [1, 0, 0]
        w = np.cross(X, vector)
        ang = np.arccos(np.dot(X, vector))
        quat = transforms3d.quaternions.axangle2quat(w, ang)
        R_q = transforms3d.quaternions.quat2mat(quat)
        T_q = transforms3d.affines.compose([0,0,0], R_q, [1,1,1])
        A = np.dot(B, T_q)
        RR = A[:3,:3]
        r = transforms3d.quaternions.mat2quat(RR)
        print(r)

        camera.transform.rotation.x = r[1]
        camera.transform.rotation.y = r[2]
        camera.transform.rotation.z = r[3]
        camera.transform.rotation.w = r[0]
    
    	# Send the transformation
        self.tf_broadcaster.sendTransform(object)
        self.tf_broadcaster.sendTransform(robot)
        self.tf_broadcaster.sendTransform(camera)


def main():
    rclpy.init()
    node = SolutionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Rest of your code



if __name__ == '__main__':
    main()
