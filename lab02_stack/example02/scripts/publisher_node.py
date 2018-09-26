#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def talker():

    pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0

    num_msg = Float64MultiArray()
    num_msg.data = []

    while not rospy.is_shutdown():

        num_msg.data.append(count)

        pub.publish(num_msg)
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass