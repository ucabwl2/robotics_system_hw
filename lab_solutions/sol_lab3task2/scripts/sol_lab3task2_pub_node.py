#!/usr/bin/env python

import rospy
import random
import numpy as np
##TODO: Include the library for an array of double message (Float64MultiArray)
from std_msgs.msg import Float64MultiArray
import pdb

def rot_publisher():

    rospy.init_node('rotate_point_publisher', anonymous=True)
    ##TODO: Initialise the publisher and publishes to the topic 'point_and_rot'
    pub = rospy.Publisher('point_and_rot',Float64MultiArray,queue_size=1000)
    rate = rospy.Rate(10) # 10hz
    ##TODO: Generate a random point and a random quaternion. Make sure that the quaternion (qx, qy, qz, qw) is of unit norm.
    ##TODO: Publish the message in the format of (px, py, pz, qx, qy, qz, qw)
    while not rospy.is_shutdown():
    	pos = np.random.rand(3)
    	quat = np.random.rand(4) #->w,x,y,z
        quat_unit = quat/np.linalg.norm(quat)
        data=np.concatenate((pos,quat_unit))
        data_for_publish=Float64MultiArray(data=data)        
        pub.publish(data_for_publish)
        rate.sleep()

if __name__ == '__main__':
    try:
        rot_publisher()
    except rospy.ROSInterruptException:
        pass
