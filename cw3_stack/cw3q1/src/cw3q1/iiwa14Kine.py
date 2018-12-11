#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion


class iiwa14_kinematic(object):

    def __init__(self):

        self.X_alpha = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, 0.0]
        self.Y_alpha = [pi, pi, 0, pi, 0, pi, 0]
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min = [-170 * pi / 180, -120 * pi / 180, -170 * pi / 180, -120 * pi / 180, -170 * pi / 180,
                                -120 * pi / 180, -175 * pi / 180]
        self.joint_limit_max = [170 * pi / 180, 120 * pi / 180, 170 * pi / 180, 120 * pi / 180, 170 * pi / 180,
                                120 * pi / 180, 175 * pi / 180]

        ##The translation between each joint for manual forward kinematic (not using the DH convention).
        self.translation_vec = np.array([[0, 0, 0.2025],
                                         [0, 0.2045, 0],
                                         [0, 0, 0.2155],
                                         [0, 0.1845, 0],
                                         [0, 0, 0.2155],
                                         [0, 0.081, 0],
                                         [0, 0, 0.045]])

        ##The centre of mass of each link with respect to the preceding joint.
        self.link_cm = np.array([[0, -0.03, 0.12],
                                 [0.0003, 0.059, 0.042],
                                 [0, 0.03, 0.13],
                                 [0, 0.067, 0.034],
                                 [0.0001, 0.021, 0.076],
                                 [0, 0.0006, 0.0004],
                                 [0, 0, 0.02]])

        ##The mass of each link.
        self.mass = [4, 4, 3, 2.7, 1.7, 1.8, 0.3]

        ##Moment on inertia of each link, defined at the centre of mass.
        ##Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
        self.Ixyz = np.array([[0.1, 0.09, 0.02],
                              [0.05, 0.018, 0.044],
                              [0.08, 0.075, 0.01],
                              [0.03, 0.01, 0.029],
                              [0.02, 0.018, 0.005],
                              [0.005, 0.0036, 0.0047],
                              [0.001, 0.001, 0.001]])

        ##gravity
        self.g = 9.8

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                queue_size=5)

        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def rotmat2q(self, T):
        q = Quaternion()

        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        if (xr == 0) and (yr == 0) and (zr == 0):
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
        else:

            x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            q.w = np.cos(angle / 2)
            q.x = x * np.sin(angle / 2)
            q.y = y * np.sin(angle / 2)
            q.z = z * np.sin(angle / 2)
        return q

    def joint_state_callback(self, msg):
        for i in range(0, 7):
            self.current_joint_position[i] = msg.position[i]

        current_pose = self.forward_kine(self.current_joint_position, 7)
        self.broadcast_pose(current_pose)

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

    def broadcast_pose(self, pose):

        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_ee'

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    ##Transformation functions for forward kinematic.
    def T_translation(self, t):
        T = np.identity(4)
        for i in range(0, 3):
            T[i, 3] = t[i]
        return T

    ##Transformation functions for forward kinematic.
    def T_rotationZ(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        return T

    ##Transformation functions for forward kinematic.
    def T_rotationX(self, theta):
        T = np.identity(4)
        T[1, 1] = np.cos(theta)
        T[1, 2] = -np.sin(theta)
        T[2, 1] = np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T


    ##Transformation functions for forward kinematic.
    def T_rotationY(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 2] = np.sin(theta)
        T[2, 0] = -np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T


    def forward_kine(self, joint, frame):
        T = np.identity(4)

        ##Add offset from the iiwa platform.
        T[2, 3] = 0.1575

        ##Manual forward kine for dynamics purpose. This chain of transformation works exactly the same as forward kinematic.
        for i in range(0, frame):
            T = T.dot(self.T_rotationZ(joint[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))

        return T


    def forward_kine_cm(self, joint, frame):
        ##TODO: Modify this function to complete the question 1.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_1' frame to the centre of mass of a link.
        raise NotImplementedError()  # Remove this line, once implemented everything

    def get_jacobian(self, joint):
        ##TODO: Modify this function to complete the question 1.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at each frame.
        raise NotImplementedError() #Remove this line, once implemented everything

    def get_jacobian_cm(self, joint, frame):
        ##TODO: Modify this function to complete the question 1.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the centre of mass of a link.
        raise NotImplementedError() #Remove this line, once implemented everything


    def inverse_kine_ite(self, desired_pose, current_joint):
        ##TODO: Modify this function to complete the question 1.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## "current_joint" is an array of double consisting of the joint value (works as a starting point for the optimisation).
        ## The output is numpy vector containing an optimised joint configuration for the desired pose.
        raise NotImplementedError() #Remove this line, once implemented everything


    def inverse_kine_closed_form(self, desired_pose):
        ##TODO: Modify this function to complete the question 1.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## The output is a numpy matrix consisting of the joint value for the desired pose.
        ## You may need to re-structure the input of this function.
        raise NotImplementedError() #Remove this line, once implemented everything


    def getB(self, joint):
        ##TODO: Modify this function to complete the question 1.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 7*7 matrix.
        raise NotImplementedError()

    def getC(self, joint, vel):
        ##TODO: Modify this function to complete the question 1.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "vel" is a numpy array of double consisting of the joint velocity.
        ## The output is a numpy 7*7 matrix.
        raise NotImplementedError()

    def getG(self, joint):
        ##TODO: Modify this function to complete the question 1.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy array 7*1.
        raise NotImplementedError()
