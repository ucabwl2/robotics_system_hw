#!/usr/bin/env python


import rospy
import numpy as np
import timeit
import math
##TODO: include the library for a msg for an array of doulbe
from std_msgs.msg import Float64MultiArray
##TODO: Complete the callback function. Your callback function should rotate the input point and print the result out on the screen.
## Show the computational time on the screen.
import pdb

def subscribeCallback(msg):
    start = rospy.get_time()

    data=msg.data

    q0=data[3]
    q1=data[4]
    q2=data[5]
    q3=data[6]
    q1s=math.pow(q1,2)
    q2s=math.pow(q2,2)
    q3s=math.pow(q3,2)

    p1=data[0]
    p2=data[1]
    p3=data[2]

    p1rot=(1-2*q2s-2*q3s)*p1 + 2*(q1*q2+q0*q3)*p2 + 2*(q1*q3-q0*q2)*p3
    p2rot=2*(q1*q2-q0*q3)*p1 + (1-2*q1s-2*q3s)*p2 + 2*(q2*q3+q0*q1)*p3
    p3rot=2*(q1*q3+q0*q2)*p1 + 2*(q2*q3-q0*q1)*p2 + (1-2*q1s-2*q2s)*p3

    
    end = rospy.get_time()
    dt=end - start
    print 'time elapsed: '+ str(dt)

    print 'The input point'
    print p1
    print p2
    print p3

    print 'The output point'
    print p1rot
    print p2rot
    print p3rot


def rot_converter():

    rospy.init_node('rotate_point_subscriber', anonymous=True)
    ##TODO: Define your subscriber and link this with the callback function.
    rospy.Subscriber("point_and_rot", Float64MultiArray, subscribeCallback)
    rate = rospy.Rate(10)

    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    try:
        rot_converter()
    except rospy.ROSInterruptException:
        pass