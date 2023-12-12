#!/usr/bin/env python3
import numpy
import random
import sys

import moveit_msgs.msg
import moveit_msgs.srv
import rclpy
from rclpy.node import Node
import rclpy.duration
import transforms3d._gohlketransforms as tf
import transforms3d
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import copy
import math

def convert_to_message(T):
    t = Pose()
    position, Rot, _, _ = transforms3d.affines.decompose(T)
    orientation = transforms3d.quaternions.mat2quat(Rot)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]        
    return t

def message_to_transform(transform_msg):
        position = transform_msg.translation
        orientation = transform_msg.rotation
        T = transforms3d.affines.compose([position.x, position.y, position.z],
                                         transforms3d.quaternions.quat2mat([orientation.w, orientation.x, orientation.y, orientation.z]),
                                         [1, 1, 1])
        return T

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        #Loads the robot model, which contains the robot's kinematics information
        self.ee_goal = None
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        #Loads the robot model, which contains the robot's kinematics information
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)
        self.base = self.robot.get_root()
        self.get_joint_info()


        self.service_cb_group1 = MutuallyExclusiveCallbackGroup()
        self.service_cb_group2 = MutuallyExclusiveCallbackGroup()
        self.q_current = []

        # Wait for moveit IK service
        self.ik_service = self.create_client(moveit_msgs.srv.GetPositionIK, '/compute_ik', callback_group=self.service_cb_group1)
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service ready')

        # Wait for validity check service
        self.state_valid_service = self.create_client(moveit_msgs.srv.GetStateValidity, '/check_state_validity',
                                                      callback_group=self.service_cb_group2)
        while not self.state_valid_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state validity service...')
        self.get_logger().info('State validity service ready')

        # MoveIt parameter
        self.group_name = 'arm'
        self.get_logger().info(f'child map: \n{self.robot.child_map}')

        #Subscribe to topics
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.get_joint_state, 10)
        self.goal_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_goal = self.create_subscription(Transform, '/motion_planning_goal', self.motion_planning_cb, 2,
                                                 callback_group=self.goal_cb_group)
        self.current_obstacle = "NONE"
        self.sub_obs = self.create_subscription(String, '/obstacle', self.get_obstacle, 10)

        #Set up publisher
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.motion_planning_timer, callback_group=self.timer_cb_group)

    
    def get_joint_state(self, msg):
        '''This callback provides you with the current joint positions of the robot 
        in member variable q_current.
        '''
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    
    def get_obstacle(self, msg):
        '''This callback provides you with the name of the current obstacle which
        exists in the RVIZ environment. Options are "None", "Simple", "Hard",
        or "Super". '''
        self.current_obstacle = msg.data

    def motion_planning_cb(self, ee_goal):
        self.get_logger().info("Motion planner goal received.")
        if self.ee_goal is not None:
            self.get_logger().info("Motion planner busy. Please try again later.")
            return
        self.ee_goal = ee_goal

    def motion_planning_timer(self):
        if self.ee_goal is not None:
            self.get_logger().info("Calling motion planner")            
            self.motion_planning(self.ee_goal)
            self.ee_goal = None
            self.get_logger().info("Motion planner done")            
                
   
    def motion_planning(self, ee_goal: Transform): 
        '''Callback function for /motion_planning_goal. This is where you will
        implement your RRT motion planning which is to generate a joint
        trajectory for your manipulator. You are welcome to add other functions
        to this class (i.e. an is_segment_valid" function will likely come in 
        handy multiple times in the motion planning process and it will be 
        easiest to make this a seperate function and then call it from motion
        planning). You may also create trajectory shortcut and trajectory 
        sample functions if you wish, which will also be called from the 
        motion planning function.

        Args: 
            ee_goal: Transform() object describing the desired base to 
            end-effector transformation 
        '''
        
        # TODO: implement motion_planning here
       
        q_goal = self.IK(message_to_transform(ee_goal))

        if len(q_goal) == 0 or not self.is_state_valid(q_goal):
            self.get_logger().info('Invalid or no IK solution found.')
            return

        tree = [RRTBranch(None, self.q_current)]
        
        MAX_ITERATIONS = 1000
        step_size = 0.1

        # RRT algorithm loop
        for _ in range(MAX_ITERATIONS):
            # Sample a random point in the configuration space
            q_rand = self.sample_random_configuration()

            # Find the closest point in the tree to the random point
            closest_branch = self.find_closest_point_in_tree(tree, q_rand)

            # Extend the tree towards q_rand
            new_branch = self.extend_tree(tree, q_rand, step_size)

            if numpy.linalg.norm((numpy.array(tree[-1].q)) - q_goal) < step_size:
                self.get_logger().info("RRT: Goal reached.")
                break

        # Shortcut and resample the path
        path = self.shortcut_and_resample(tree, q_goal)
        self.publish_trajectory(path)
        
    def sample_random_configuration(self):
        # Implement random sampling within joint limits [-pi, pi]
        return [random.uniform(-math.pi, math.pi) for _ in range(self.num_joints)]

    def extend_tree(self, tree, q_rand, step_size=0.1):
        # Find the closest branch in the tree to the random configuration
        closest_branch = min(tree, key=lambda branch: numpy.linalg.norm(numpy.array(branch.q) - numpy.array(q_rand)))
        q_closest = numpy.array(closest_branch.q)

        # Compute the direction from the closest branch to the random configuration
        direction = numpy.array(q_rand) - q_closest
        distance = numpy.linalg.norm(direction)

        # If the random point is within the step size, add it directly to the tree
        if distance < step_size:
            new_branch = RRTBranch(closest_branch, q_rand)
            tree.append(new_branch)
            return new_branch

        # Otherwise, extend the tree towards the random point by the step size
        direction = direction / distance  # Normalize the direction vector
        q_new = q_closest + direction * step_size

        # Ensure that the new configuration is within joint limits and not in collision
        if self.is_state_valid(q_new):
            new_branch = RRTBranch(closest_branch, q_new.tolist())
            tree.append(new_branch)
            return new_branch
        else:
            # If the new configuration is not valid, return None
            return None


    def shortcut_and_resample(self, tree, q_goal):
        path = [q_goal]
        current_branch = tree[-1]

        while current_branch:
            path.insert(0, current_branch.q)
            current_branch = current_branch.parent

        resampled_path = [path[0]]
        for i in range(1, len(path)):
            q_prev, q_curr = path[i - 1], path[i]
            substract = numpy.array(q_curr) - numpy.array(q_prev)
            distance = numpy.linalg.norm(substract)
            num_samples = max(int(distance / 0.5), 1)
            for j in range(1, num_samples + 1):
                resampled_point = q_prev + (j / num_samples)*(substract)
                resampled_path.append(resampled_point)
        return resampled_path


    def publish_trajectory(self, path):
        # Convert the path to a JointTrajectory message and publish it
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        # path = numpy.array(path, dtype=numpy.float64)
        point = JointTrajectoryPoint()
        point.positions = path[0]
        trajectory_msg.points.append(point)




        for i in range(1,len(path)):
            point = JointTrajectoryPoint()
            point.positions = path[i].tolist()
            trajectory_msg.points.append(point)
            # point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        self.pub.publish(trajectory_msg)

        



    def find_closest_point_in_tree(self, tree, r):
        r_array = numpy.array(r)
        shortest_distance = numpy.linalg.norm(r_array - numpy.array(tree[0].q))
        # shortest_distance = numpy.linalg.norm(r-tree[0].q)
        closest_point = tree[0]
        for i in range(1, len(tree)-1):
            temp = r_array - numpy.array(tree[i].q)
            if shortest_distance > numpy.linalg.norm(temp):
                shortest_distance = numpy.linalg.norm(temp)
                closest_point = tree[i]
        return closest_point


    
    def IK(self, T_goal):
        """ This function will perform IK for a given transform T of the 
        end-effector. It .

        Returns:
            q: returns a list q[] of values, which are the result 
            positions for the joints of the robot arm, ordered from proximal 
            to distal. If no IK solution is found, it returns an empy list
        """

        req = moveit_msgs.srv.GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = 'base'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        
        self.get_logger().info('Sending IK request...')
        res = self.ik_service.call(req)
        self.get_logger().info('IK request returned')
        
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        for i in range(0,len(q)):
            while (q[i] < -math.pi): q[i] = q[i] + 2 * math.pi
            while (q[i] > math.pi): q[i] = q[i] - 2 * math.pi
        return q

    
    def get_joint_info(self):
        '''This is a function which will collect information about the robot which
        has been loaded from the parameter server. It will populate the variables
        self.num_joints (the number of joints), self.joint_names and
        self.joint_axes (the axes around which the joints rotate)
        '''
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
        self.get_logger().info('Num joints: %d' % (self.num_joints))


    
    def is_state_valid(self, q):
        """ This function checks if a set of joint angles q[] creates a valid state,
        or one that is free of collisions. The values in q[] are assumed to be values
        for the joints of the UR5 arm, ordered from proximal to distal.

        Returns:
            bool: true if state is valid, false otherwise
        """
        req = moveit_msgs.srv.GetStateValidity.Request()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names

        req.robot_state.joint_state.position = list(q)
        req.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()

        res = self.state_valid_service.call(req)

        return res.valid



class RRTBranch(object):
    '''This is a class which you can use to keep track of your tree branches.
    It is easiest to do this by appending instances of this class to a list 
    (your 'tree'). The class has a parent field and a joint position field (q). 
    
    You can initialize a new branch like this:
        RRTBranch(parent, q)
    Feel free to keep track of your branches in whatever way you want - this
    is just one of many options available to you.
    '''
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


def main(args=None):
    rclpy.init(args=args)
    ma = MoveArm()
    ma.get_logger().info("Move arm initialization done")
    executor = MultiThreadedExecutor()
    executor.add_node(ma)
    executor.spin()
    ma.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

