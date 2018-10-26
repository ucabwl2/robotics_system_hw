#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class hArm_kinematic(object):

    def __init__(self):
        self.dh_params = [[0.0, -pi/2, 0.126, 0.0],
                          [0.26569, 0.0, 0.0, -pi/2],
                          [0.03, -pi/2, 0.0, 0.0],
                          [0.0, -pi/2, 0.258, 0.0],
                          [0.0, -pi/2, 0.0, 0.0],
                          [0.0, 0.0, 0.0, 0.0]]

        self.joint_offset = [170*pi/180, 65*pi/180, -146*pi/180, 102.5*pi/180, 167.5*pi/180]

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min = [-pi, -pi/2, -pi/2, -pi, -pi/2, -pi]
        self.joint_limit_max = [pi, pi/2, 3*pi/4, pi, pi/2, pi]

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)
        self.traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def dh_matrix_standard(self, a, alpha, d, theta):
        A = np.zeros((4, 4))

        A[0, 0] = np.cos(theta)
        A[0, 1] = -np.sin(theta) * np.cos(alpha)
        A[0, 2] = np.sin(theta) * np.sin(alpha)
        A[0, 3] = a * np.cos(theta)

        A[1, 0] = np.sin(theta)
        A[1, 1] = np.cos(theta) * np.cos(alpha)
        A[1, 2] = -np.cos(theta) * np.sin(alpha)
        A[1, 3] = a * np.sin(theta)

        A[2, 1] = np.sin(alpha)
        A[2, 2] = np.cos(alpha)
        A[2, 3] = d

        A[3, 3] = 1.0

        return A


    def joint_state_callback(self, msg):
        for i in range(0, 6):
            self.current_joint_position[i] = msg.position[i]


    def forward_kine(self, joint, frame):
        T = np.identity(4)

        for i in range(0, 6):
            A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], joint[i] + self.dh_params[i][3])


            T = T.dot(A)

        return T

    def publish_joint_trajectory(self, joint_trajectory, tfs):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_trajectory
        point.time_from_start.nsecs = tfs

        msg.points.append(point)

        self.traj_publisher.publish(msg)


    def inverse_kine_closed_form(self, pose):

        ##There are 8 possible inverse kinematic solutions. The last column indicates whether or not the solution lies within the robot workspace (does not exceed joint limit)
        inv_kine_sol = np.zeros((8, 7))

        a2 = self.dh_params[1, 0]
        a3 = self.dh_params[2, 0]
        d4 = self.dh_params[3, 2]
        beta = np.arctan2(d4, a3)

        ##Solve for theta1
        inv_kine_sol[0, 0] = inv_kine_sol[1, 0] = inv_kine_sol[2, 0] = inv_kine_sol[3, 0] = np.arctan2(pose[1, 3], pose[0, 3])

        ##Check with the joint limit for the other solution
        if (inv_kine_sol[0, 0] <= 0):
            inv_kine_sol[4, 0] = inv_kine_sol[5, 0] = inv_kine_sol[6, 0] = inv_kine_sol[7, 0] = np.arctan2(pose[1, 3], pose[0, 3]) + pi
        else:
            inv_kine_sol[4, 0] = inv_kine_sol[5, 0] = inv_kine_sol[6, 0] = inv_kine_sol[7, 0] = np.arctan2(pose[1, 3], pose[0, 3]) - pi


        ##Solve for theta3
        for i in range(0, 2):
            pose16 = np.linalg.inv(self.dh_matrix_standard(self.dh_params[0, 0], self.dh_params[0, 1], self.dh_params[0, 2],
                                                           self.dh_params[0, 3] + inv_kine_sol[4*i, 0])).dot(pose)

            K = (pose16[0, 3]**2 + pose16[1, 3]**2 - a3**2 - a2**2 - d4**2)/(2*a2*np.sqrt(a3**2 + d4**2))
            inv_kine_sol[4*i, 2] = inv_kine_sol[4*i + 1, 2] = np.arccos(K) - beta
            inv_kine_sol[4*i + 2, 2] = inv_kine_sol[4*i + 3, 2] = -np.arccos(K) - beta

            A1 = a2 + a3*np.cos(inv_kine_sol[4*i, 2]) - d4*np.sin(inv_kine_sol[4*i, 2])
            A2 = a2 + a3*np.cos(inv_kine_sol[4*i + 2, 2]) - d4*np.sin(inv_kine_sol[4*i + 2, 2])

            B1 = a3*np.sin(inv_kine_sol[4*i, 2]) + d4*np.cos(inv_kine_sol[4*i, 2])
            B2 = a3*np.sin(inv_kine_sol[4*i + 2, 2]) + d4*np.cos(inv_kine_sol[4*i + 2, 2])


            ##Solve for theta2
            inv_kine_sol[4*i, 1] = inv_kine_sol[4*i + 1, 1] = np.arctan2(pose16[0, 3], pose16[1, 3]) - np.arctan2(B1, A1)
            inv_kine_sol[4*i + 2, 1] = inv_kine_sol[4*i + 3, 1] = np.arctan2(pose16[0, 3], pose16[1, 3]) - np.arctan2(B2, A2)


        for i in range(0, 7):
            joint13 = [inv_kine_sol[i, 0], inv_kine_sol[i, 1], inv_kine_sol[i, 2]]
            pose46 = np.linalg.inv(self.forward_kine(joint13, 3)).dot(pose)

            ##Solve for theta5
            if (i%2 == 0):
                inv_kine_sol[i, 4] = np.arccos(-pose46[2, 2])
            else:
                inv_kine_sol[i, 4] = -np.arccos(-pose46[2, 2])

            ##Solve for theta4
            inv_kine_sol[i, 3] = np.arctan2(pose46[1, 2]/(-np.sin(inv_kine_sol[i, 4])),
                                            pose46[0, 2]/(-np.sin(inv_kine_sol[i, 4])))

            ##Solve for theta6
            inv_kine_sol[i, 5] = np.arctan2(pose46[2, 1]/np.sin(inv_kine_sol[i, 4]),
                                            pose46[2, 0]/(-np.sin(inv_kine_sol[i, 4])))

        ##Check if the inverse kinematic solutions are in the robot workspace
        for i in range(0, 8):
            for j in range(0, 6):
                if (inv_kine_sol[i, j] <= self.joint_limit_min[j]) or (inv_kine_sol[i, j] >= self.joint_limit_max[j]):
                    ##Put some value there to let you know that this solution is not valid.
                    inv_kine_sol[i, 6] = -1
                    break
                else:
                    ##Put some value there to let you know that this solution is valid.
                    inv_kine_sol[i, 6] = 1


        return inv_kine_sol