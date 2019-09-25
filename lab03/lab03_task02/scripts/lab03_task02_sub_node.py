#!/usr/bin/env python

import rospy
import numpy as np
##TODO: include the library for a msg for an array of doulbe
##TODO: Complete the callback function. Your callback function should rotate the input point and print the result out on the screen.
## Show the computational time on the screen.

def subscribeCallback(msg):
    print  'test'


def rot_converter():

    rospy.init_node('rotate_point_subscriber', anonymous=True)
    ##TODO: Define your subscriber and link this with the callback function.
    rate = rospy.Rate(10)

    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    try:
        rot_converter()
    except rospy.ROSInterruptException:
        pass