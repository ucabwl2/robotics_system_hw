#include "ros/ros.h"
#include "std_msgs/String.h"

void subscribeCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Incoming message: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/chatter", 1000, subscribeCallback);
    ros::spin();

    return 0;
}