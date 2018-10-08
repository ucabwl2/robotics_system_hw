#!/usr/bin/env python

import rospy
import random
##TODO: Include the library for an array of double message (Float64MultiArray)

def rot_publisher():

    rospy.init_node('rotate_point_publisher', anonymous=True)
    ##TODO: Initialise the publisher and publishes to the topic 'point_and_rot'
    ##TODO: Generate a random point and a random quaternion. Make sure that the quaternion (qx, qy, qz, qw) is of unit norm.
    ##TODO: Publish the message in the format of (px, py, pz, qx, qy, qz, qw)

if __name__ == '__main__':
    try:
        rot_publisher()
    except rospy.ROSInterruptException:
        pass