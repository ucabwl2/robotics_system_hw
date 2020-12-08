#!/usr/bin/env python

import rospy
import math
##TODO: include the library for a msg typed VectorStamped

def talker():

    ##TODO: Define a publisher
    pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    radius = 1.0
    period = 5.0

    ##TODO: Define your message

    starting_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():

        ##TODO: Input the data into your message. Your message should be a position of a unit circle on the XZ-plane.
        msg.header.stamp = rospy.Time.now()

        msg.point.x = radius * math.cos(2 * math.pi * (msg.header.stamp.to_sec() - starting_time)/period)
        msg.point.y = radius * math.sin(2 * math.pi * (msg.header.stamp.to_sec() - starting_time)/period)
        msg.point.z = 0.0

        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
