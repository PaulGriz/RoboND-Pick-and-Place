#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
from mpmath import radians
from sympy import Matrix, symbols, sqrt, cos, sin, atan2, acos, pi
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import rospy


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        # To prevent complex number errors & improve readability,
        # half_pi is used over pi / 2
        half_pi = (pi / 2)

        # Symbols used for theta1, theta2, & theta3
        q1, q2, q3 = symbols('q1:4')

        # Solving the Kinematics using a "closed-form" solution
        # The advantage of such a design is that it kinematically
        # separates the position and orientation of the end effector.

        # Instead of solving twelve nonlinear equations simultaneously
        # (one equation for each term in the first three rows of the overall homogeneous
        # transform matrix),it is now possible to independently solve two simpler problems

        # The Cartesian coordinates of the wrist center.
        # A spherical wrist uses the first three joints to control the position of the wrist center.
        # So, only Homogeneous Transforms from Joints 1, 2, 3 is required.
        T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0],
                       [sin(q1), cos(q1), 0, 0],
                       [0, 0, 1, 0.75],
                       [0, 0, 0, 1]])

        T1_2 = Matrix([[cos(q2 - half_pi), -sin(q2 - half_pi), 0, 0.35],
                       [0, 0, 1, 0],
                       [-sin(q2 - half_pi), -cos(q2 - half_pi), 0, 0],
                       [0, 0, 0, 1]])

        T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25],
                       [sin(q3), cos(q3), 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        # Symbols for the Roll, Pitch, & Yaw
        r, p, y = symbols('r p y')

        ###### ROLL ######
        ROT_x = Matrix([[1, 0, 0],
                        [0, cos(r), -sin(r)],
                        [0, sin(r), cos(r)]])
        ###### PITCH ######
        ROT_y = Matrix([[cos(p), 0, sin(p)],
                        [0, 1, 0],
                        [-sin(p), 0, cos(p)]])
        ###### YAW ######
        ROT_z = Matrix([[cos(y), -sin(y), 0],
                        [sin(y), cos(y), 0],
                        [0, 0, 1]])

        ROT_EE = ROT_z * ROT_y * ROT_x

        # Rotation error
        Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

        ROT_EE = ROT_EE * Rot_Error

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            ########### Calculating The Wrist Center's Position ###########

            # Extract end-effector position and orientation from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px], [py], [pz]])
            # Wrist Center = WC
            # --> WC - X position = WC[0]
            # --> WC - Y position = WC[1]
            # --> WC - Z position = WC[2]
            # Since WC is in the same position as Joint 5, the length
            #   of (Joint 5 + Joint 6) must be subtracted.
            # From the urdf file:
            #   Joint 5 has length of 0.11m
            #   Joint 6 has length of 0.196m
            # So, subtracting (0.11 + 0.196) = 0.303
            WC = EE - (0.303) * ROT_EE[:, 2]
            ##### --> End of finding the Wrist Center's Position #####

            ########### Calculating The Wrist Center's Orientation ###########
            # Variable names refer to the chart at "./diagrams/Orientation_Chart.png"
            link_1 = 1.501
            # The length of a vector from Joint-2 to WC or Joint-5
            J2_5_len = round(sqrt(pow(
                sqrt((WC[0] * WC[0]) + (WC[1] * WC[1])) - 0.35, 2) + pow((WC[2] - 0.75), 2)), 3)
            link_3 = 1.25
            # Using the law of cosines, angle_a & angle_b can be found:
            #   From the link_1, link_2, and J2_5_len triangle
            angle_a = acos((J2_5_len * J2_5_len + link_3 * link_3 - link_1 * link_1)
                           / (2.0 * J2_5_len * link_3))
            angle_b = acos((link_1 * link_1 + link_3 * link_3 - J2_5_len * J2_5_len)
                           / (2.0 * link_1 * link_3))

            ############ Theta 1 ############
            theta1 = round(atan2(WC[1], WC[0]), 3)

            ############ Theta 2 ############
            # angle_c = atan2(WC[2] - 0.75,
            #                 sqrt((WC[0] * WC[0]) + (WC[1] * WC[1]) - 0.35))
            # Explicitly declaring theta2 as a float to prevent ROS message errors
            theta2 = round(float(half_pi - angle_a -
                                 atan2(WC[2] - 0.75,
                                       sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)), 3)

            ############ Theta 3 ############
            # 0.0373 = atan2(.056,1.5) accounts for sag in link_4 of -0.054m
            theta3 = round(float(half_pi - (angle_b + 0.036)), 3)

            # Joints 1, 2, & 3
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            # Setting Float precision to 5
            # chop=True used to remove roundoff errors smaller than the desired precision
            R0_3 = R0_3.evalf(5, subs={q1: theta1, q2: theta2, q3: theta3},
                              chop=True)

            ############ Theta 4, 5, 6 ############
            # ----> Euler angles
            # from rotation matrix
            R3_6 = R0_3.transpose() * ROT_EE
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]),
                           R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            joint_trajectory_point.positions = [theta1, theta2, theta3,
                                                theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" %
                      len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
