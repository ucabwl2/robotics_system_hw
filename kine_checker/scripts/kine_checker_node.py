#!/usr/bin/env python

import rospkg
from math import pi
import rosbag
import rospy
import tf2_kdl
import tf2_ros

from cw2q4.youbotKine import youbot_kinematic

if __name__ == '__main__':
    rospy.init_node('youbot_kine_node')
    youbot = youbot_kinematic()

    curr_joint = [0, 0, 0, 0, 0]
    pose1_values = [-0.411883, 1.30864, -1.96052, -1.07369, -1.16671]
    pose2_values = [-1.66061, 1.78931, -2.54818, -1.33999, 1.70616]
    pose3_values = [-0.268307, 1.48098, -2.04278, -0.859284, -2.2123]
    pose4_values = [-1.5747, 1.0566, -1.99377, -2.25452, 0.975057]
    pose5_values = [1.785, 2.0513, -2.54818, -1.34124, 0.472335]

    #pose1_values
    T = youbot.forward_kine(pose1_values, 5)
    jacobian = youbot.get_jacobian(pose1_values)
    IK_closed = youbot.inverse_kine_closed_form(T)
    IK_ite = youbot.inverse_kine_ite(T, curr_joint)
    singularity = youbot.check_singularity(pose1_values)
    print("///// JOINT 1 /////")
    print("The joint space is: ")
    print(pose1_values)
    print("Jacobian: ")
    print(jacobian)
    print("Closed form IK: ")
    print(IK_closed)
    print("Iterative form IK: ")
    print(IK_ite)
    print("Singularity checker: ")
    print(singularity)
    print("\n")

    #pose2_values
    T = youbot.forward_kine(pose1_values, 5)
    jacobian = youbot.get_jacobian(pose2_values)
    IK_closed = youbot.inverse_kine_closed_form(T)
    IK_ite = youbot.inverse_kine_ite(T, curr_joint)
    singularity = youbot.check_singularity(pose2_values)
    print("test")
    print("///// JOINT 2 /////")
    print("The joint space is: ")
    print(pose2_values)
    print("Jacobian: ")
    print(jacobian)
    print("Closed form IK: ")
    print(IK_closed)
    print("Iterative form IK: ")
    print(IK_ite)
    print("Singularity checker: ")
    print(singularity)
    print("\n")

    #pose3_values
    T = youbot.forward_kine(pose3_values, 5)
    jacobian = youbot.get_jacobian(pose3_values)
    IK_closed = youbot.inverse_kine_closed_form(T)
    IK_ite = youbot.inverse_kine_ite(T, curr_joint)
    singularity = youbot.check_singularity(pose3_values)
    print("///// JOINT 3 /////")
    print("The joint space is: ")
    print(pose3_values)
    print("Jacobian: ")
    print(jacobian)
    print("Closed form IK: ")
    print(IK_closed)
    print("Iterative form IK: ")
    print(IK_ite)
    print("Singularity checker: ")
    print(singularity)
    print("\n")

    #pose4_values
    T = youbot.forward_kine(pose4_values, 5)
    jacobian = youbot.get_jacobian(pose4_values)
    IK_closed = youbot.inverse_kine_closed_form(T)
    IK_ite = youbot.inverse_kine_ite(T, curr_joint)
    singularity = youbot.check_singularity(pose4_values)
    print("///// JOINT 4 /////")
    print("The joint space is: ")
    print(pose4_values)
    print("Jacobian: ")
    print(jacobian)
    print("Closed form IK: ")
    print(IK_closed)
    print("Iterative form IK: ")
    print(IK_ite)
    print("Singularity checker: ")
    print(singularity)
    print("\n")

    #pose5_values
    T = youbot.forward_kine(pose5_values, 5)
    jacobian = youbot.get_jacobian(pose5_values)
    IK_closed = youbot.inverse_kine_closed_form(T)
    IK_ite = youbot.inverse_kine_ite(T, curr_joint)
    singularity = youbot.check_singularity(pose5_values)
    print("///// JOINT 5 /////")
    print("The joint space is: ")
    print(pose5_values)
    print("Jacobian: ")
    print(jacobian)
    print("Closed form IK: ")
    print(IK_closed)
    print("Iterative form IK: ")
    print(IK_ite)
    print("Singularity checker: ")
    print(singularity)
    print("\n")
