#!/usr/bin/env python

import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('robotis_tf_listener_node')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        try:
            T = tfBuffer.lookup_transform('world', 'link6', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            rate.sleep()
            continue

        print T

        rate.sleep()