#!/usr/bin/env python3
import sys
sys.path.append(".")

import numpy
import rclpy
import transforms3d._gohlketransforms as tf
from .sys_urdf_setup import CartesianGrader
import time

def get_franka_poses():
   trans1 = (0.3, 0.2, 0.5)
   rot1 = (0.0, 0.0, 1, 0.0)
   trans2 = (0.3, 0.2, 1.0)
   rot2 = (0.0, 0.0, 0.0, 1.0)
   trans3 = (0.3, 0.1, 0.8)
   rot3 = (0.87, 0.0, 0.47, 0.0)
   trans4 = (-0.3, 0.3, 0.2)
   rot4 = (0.87, 0.0, 0.47, 0.0)

   return (trans1, rot1, trans2, rot2, trans3, rot3, trans4, rot4)

def get_ur5_poses():
   trans1 = (-0.4, 0.4, 0.1)
   rot1 = (-0.03218073733996252, -0.014727757546290993, -0.415886141536534, -0.9087277978469505)
   trans2 = (-0.4, 0.4, 0.1)
   rot2 = (0.7, 0.5, 0.5, -0.3)
   trans3 = (-0.1, 0.6, 0.4)
   rot3 = (0.0, 0.7, 0.6, -0.3)
   trans4 = (-0.3, -0.2, 0.3)
   rot4 = (-0.1, 0.7, 0.6, -0.3)

   return (trans1, rot1, trans2, rot2, trans3, rot3, trans4, rot4)


def perform_ik_poses(robot, cg):
    ik_grade = 0

    if robot == 'ur': (trans1, rot1, trans2, rot2, trans3, rot3, trans4, rot4) = get_ur5_poses()
    else: (trans1, rot1, trans2, rot2, trans3, rot3, trans4, rot4) = get_franka_poses()

    cg.get_logger().info("Performing IK")
    trans = tf.translation_matrix(trans1)
    rot = tf.quaternion_matrix(rot1)
    ik_grade += cg.go_to_ik_pose("IK1", numpy.dot(trans,rot), 10)

    trans = tf.translation_matrix(trans2)
    rot = tf.quaternion_matrix(rot2)
    ik_grade += cg.go_to_ik_pose("IK2", numpy.dot(trans,rot), 10)

    trans = tf.translation_matrix(trans3)
    rot = tf.quaternion_matrix(rot3)
    ik_grade += cg.go_to_ik_pose("IK3", numpy.dot(trans,rot), 10)

    trans = tf.translation_matrix(trans4)
    rot = tf.quaternion_matrix(rot4)
    ik_grade += cg.go_to_ik_pose("IK4", numpy.dot(trans,rot), 10)

    return ik_grade


def perform_cartesian_poses(robot, cg):
    cc_grade = 0

    if robot == 'ur': (trans1, rot1, trans2, rot2, trans3, rot3, trans4, rot4) = get_ur5_poses()
    else: (trans1, rot1, trans2, rot2, trans3, rot3, trans4, rot4) = get_franka_poses()

    cg.get_logger().info("Testing cartesian control")
    trans = tf.translation_matrix(trans1)
    rot = tf.quaternion_matrix(rot1)
    cc_grade += cg.go_to_pose("CC1", numpy.dot(trans,rot), False, 0.0, 10, 1.5)		#Translation only

    trans = tf.translation_matrix(trans2)
    rot = tf.quaternion_matrix(rot2)
    cc_grade += cg.go_to_pose("CC2", numpy.dot(trans,rot), False, 0.0, 10, 1.5)		#Rotation only

    trans = tf.translation_matrix(trans3)
    rot = tf.quaternion_matrix(rot3)
    cc_grade += cg.go_to_pose("CC3", numpy.dot(trans,rot), False, 0.0, 10, 2)		#Translation and rotation

    #print("Moving secondary objective"
    cg.get_logger().info("Testing cartesian control with secondary objective")
    trans = tf.translation_matrix(trans3)
    rot = tf.quaternion_matrix(rot3)
    secondary_obj = cg.go_to_pose("SO1", numpy.dot(trans,rot), True, 1.5, 20, 2.5)		#Secondary objective, no movement

    return cc_grade, secondary_obj


def main(args=None):
    time.sleep(1.0)
    rclpy.init(args=args)
    cg = CartesianGrader()
    cg.get_logger().info("Initialization done")
    robot = cg.robot.name
    time.sleep(1.0)
    cg.get_logger().info("Resetting robot")
    cg.reset_robot()
    time.sleep(1.0)

    grade = 0
    ik_grade = perform_ik_poses(robot, cg)
    if ik_grade ==4:
        grade+=2.5
        cg.get_logger().info("IK: PASSED")
    else:
        ik_grade2 = perform_ik_poses(robot, cg)
        if ik_grade2 ==4:
            cg.get_logger().info("IK: PASSED")
            grade+=2.5
        else: cg.get_logger().info("IK FAILED")

    time.sleep(1.0)
    cg.reset_robot()
    time.sleep(1.0)

    cc_grade, secondary_obj = perform_cartesian_poses(robot, cg)
    if cc_grade == 5:
        cg.get_logger().info("Cartesian control: PASSED")
    else:
        cg.get_logger().info("Cartesian control: FAILED")
        if cc_grade == 0:
            cg.get_logger().info("BOTH rotation and translation failed")
        elif cc_grade == 3:
            cg.get_logger().info("SIMULTANEOUS rotation and translation failed")
        else:
            cg.get_logger().info("EITHER rotation OR translation failed")

    if secondary_obj==2.5:
        cg.get_logger().info("Secondary objective: PASSED")
    else: cg.get_logger().info("Secondary objective: FAILED")

    grade += cc_grade
    grade += secondary_obj

    cg.get_logger().info(f"Grade: {grade}")
    cg.listener.dedicated_listener_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
