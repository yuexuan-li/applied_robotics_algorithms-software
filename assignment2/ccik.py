#!/usr/bin/env python3

import math
import numpy
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from custom_msg.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import transforms3d
import transforms3d._gohlketransforms as tf
from threading import Thread, Lock

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(Node):
    def __init__(self):
        super().__init__('ccik')
    #Load robot from parameter server
        # self.robot = URDF.from_parameter_server()
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        # print(robot_desription_text)
        self.robot = URDF.from_xml_string(robot_desription_text)

    #Subscribe to current joint state of the robot
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.get_joint_state, 10)

    #This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

    #This is a mutex
        self.mutex = Lock()

    #Subscribers and publishers for for cartesian control
        self.cartesian_command_sub = self.create_subscription(
            CartesianCommand, '/cartesian_command', self.get_cartesian_command, 10)
        self.velocity_pub = self.create_publisher(JointState, '/joint_velocities', 10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        self.ik_command_sub = self.create_subscription(
            Transform, '/ik_command', self.get_ik_command, 10)
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
        #self.get_logger().info(f'Joint names {self.joint_names}')
        #self.get_logger().info(f'Joint axes {self.joint_axes}')

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
        # compute transform from current end-effector pose to desired one
        #Initalize
        delta_x = numpy.zeros(6)
        self.joint_velocity_msg.name = self.joint_names
        joint_values = self.q_current
        
        #forward kinematics
        joint_transforms, b_T_ee = self.forward_kinematics(joint_values)
        
        translation_vector = numpy.array([command.x_target.translation.x, command.x_target.translation.y, command.x_target.translation.z])
        # quaternion = (command.x_target.rotation.x, command.x_target.rotation.y, command.x_target.rotation.z, command.x_target.rotation.w)
        target_rotation = transforms3d.quaternions.quat2mat([command.x_target.rotation.w, command.x_target.rotation.x, command.x_target.rotation.y, command.x_target.rotation.z])
        delta_x[:3] = translation_vector - b_T_ee[:3, 3]
        rotation_error = numpy.dot(target_rotation, b_T_ee[:3, :3].T)
        angle, axis = self.rotation_from_matrix(rotation_error)
        delta_x[3:] = axis * angle


        #Jacobian and pseudoinverse
        J = self.get_jacobian(b_T_ee, joint_transforms)
        Jpsinv = numpy.linalg.pinv(J)
        gain_p = 5
        #where q_dot = Jpsinv*Vee
        v_ee = gain_p * delta_x
        q_dot = Jpsinv @ v_ee


        #secondary objective
        if command.secondary_objective:
                null_space_projector = numpy.eye(self.num_joints) - Jpsinv @ J
                q0_dot = 3 * (command.q0_target - self.q_current[0])
                qdot_jnull = null_space_projector @ numpy.array([q0_dot] + [0] * (self.num_joints - 1))
                q_dot = q_dot + qdot_jnull
        
        self.joint_velocity_msg.name = self.joint_names
        self.joint_velocity_msg.velocity = q_dot.tolist()
        self.velocity_pub.publish(self.joint_velocity_msg)


        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
        for i in range(self.num_joints):
           z_axis = joint_transforms[i][:3, 2]
           joint_pos = joint_transforms[i][:3, 3]
           ee_pos = b_T_ee[:3, 3]
      
           J[:3, i] = numpy.cross(z_axis, (ee_pos - joint_pos))
           J[3:, i] = z_axis

        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE

            #initial
        delta_x = numpy.zeros(6)
        
        start_time = time.time()
        while time.time() - start_time <10:
        #fwk to get new values
            joint_transforms, b_T_ee = self.forward_kinematics(self.q_current)
        
        #calculate delta_x
            delta_x[:3] = [
                command.translation.x - b_T_ee[0, 3],
                command.translation.y - b_T_ee[1, 3],
                command.translation.z - b_T_ee[2, 3]
                ]
            desired_rotation = transforms3d.quaternions.quat2mat([command.rotation.w, command.rotation.x, command.rotation.y, command.rotation.z])
            rotation_error = numpy.dot(desired_rotation,b_T_ee[:3, :3].T)
            angle, axis = self.rotation_from_matrix(rotation_error)
            delta_x[3:] = axis * angle
        
        #find Jacobian and psuedoinverse
            J = self.get_jacobian(b_T_ee, joint_transforms)
                
            if numpy.linalg.norm(delta_x) < 0.001:
                self.joint_command_msg.name = self.joint_names
                self.joint_command_msg.position = [float(q) for q in self.q_current]
                self.joint_command_pub.publish(self.joint_command_msg)
                break
            damping_factor = 0.01
            JtJ = J.T @ J
            damp_JtJ = JtJ + damping_factor**2 * numpy.eye(self.num_joints)
            Jpsinv = numpy.linalg.inv(damp_JtJ) @ J.T
            q_dot = Jpsinv @ delta_x
            self.q_current += q_dot * 0.01


        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each link of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.translation_matrix(joint.origin.xyz), tf.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], 'rxyz'))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


def main(args = None):
    rclpy.init()
    ccik = CCIK()
    rclpy.spin(ccik)
    ccik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
