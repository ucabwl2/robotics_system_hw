#!/usr/bin/env python

import rospkg

import rosbag
import rospy
import tf2_kdl
import tf2_ros

from youbotkdl_tester.YoubotKDL import YoubotKDL

if __name__ == '__main__':
    rospy.init_node('youbot_bag_follower')
    youbot = YoubotKDL()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rospack = rospkg.RosPack()

    # get the file path for inverse_kinematics
    path = rospack.get_path('youbotkdl_tester')
    bag = rosbag.Bag(path + '/bags/data.bag')
    trans_list = []

    for topic, msg, t in bag.read_messages(topics=['target_tf']):
        trans_list.append(msg)

    for trans in trans_list:
        # Publish the desired tf
        trans.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(trans)

        # Find your current cartesian pose
        youbot.forward_kinematics(youbot.current_joint_position, youbot.current_pose)
        youbot.broadcast_pose(youbot.current_pose)

        # Get next desired pose from rosbag
        frame = tf2_kdl.transform_to_kdl(trans)

        # Ikine
        jointkdl = youbot.inverse_kinematics_closed(frame)
        joint_array = []

        for pos in jointkdl:
            joint_array.append(pos)

        print("Targeted transformation: ")
        print(trans)
        print("Convert to joint values: {}".format(joint_array))
        rospy.sleep(0.2)

        rospy.sleep(1)
        raw_input('Press enter for next pose\n')
