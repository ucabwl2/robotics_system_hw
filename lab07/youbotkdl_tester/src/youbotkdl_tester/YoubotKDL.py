#!/usr/bin/env python

import rospy
from math import pi
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import PyKDL
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf_conversions import posemath

import tf2_kdl


class YoubotKDL(object):
    def __init__(self):

        self.dh_params = [[-0.033, pi / 2, 0.145, pi, -1],
                          [0.155, 0, 0, pi / 2, -1],
                          [0.135, 0, 0, 0.0, -1],
                          [-0.002, pi / 2, 0, -pi / 2, -1],
                          [0, pi, -0.185, -pi, -1]]

        self.joint_offset = [170*pi/180, 65*pi/180, -146*pi/180, 102.5*pi/180, 167.5*pi/180]

        self.joint_limit_min = [-169*pi/180, -65*pi/180, -151*pi/180, -102.5*pi/180, -167.5*pi/180]
        self.joint_limit_max = [169*pi/180, 90*pi/180, 146*pi/180, 102.5*pi/180, 167.5*pi/180]

        # Setup the subscribers for the joint states
        self.subscriber_joint_state_ = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)
        # TF2 broadcaster
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

        # PyKDL
        self.kine_chain = PyKDL.Chain()
        self.setup_kdl_chain()
        self.current_joint_position = PyKDL.JntArray(self.kine_chain.getNrOfJoints())
        self.current_pose = PyKDL.Frame()

        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.kine_chain)
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)

    def setup_kdl_chain(self):
        for dh in self.dh_params:
            self.kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Vector(), PyKDL.Vector(0, 0, dh[4]),
                                                                 PyKDL.Joint.RotAxis),
                                                     PyKDL.Frame().DH(dh[0], dh[1], dh[2], dh[3])))

    def joint_state_callback(self, msg):
        # Copies joint position into KDL JntArray
        for i in range(0, 5):
            self.current_joint_position[i] = msg.position[i]

    def forward_kinematics(self, joint, pose):
        self.fk_solver.JntToCart(joint, pose)

    def broadcast_pose(self, pose):
        trans = TransformStamped()

        t = posemath.toMsg(pose)
        trans.transform.translation = t.position
        trans.transform.rotation = t.orientation
        trans.header.frame_id = "base_link"
        trans.header.stamp = rospy.Time.now()
        trans.child_frame_id = "arm_end_effector"

        self.pose_broadcaster.sendTransform(trans)
        return trans

    def get_jacobian(self, joint):
        jac = PyKDL.Jacobian(self.kine_chain.getNrOfJoints())
        self.jac_calc.JntToJac(joint, jac)
        return jac

    def inverse_kinematics_closed(self, desired_pose):

        required_joint = PyKDL.JntArray(self.kine_chain.getNrOfJoints())
        # Current joint array, desired frame, required joint
        self.ik_solver.CartToJnt(self.current_joint_position, desired_pose, required_joint)

        return required_joint