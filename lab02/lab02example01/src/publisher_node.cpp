#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/chatter", 1000);

    ros::Rate loop_rate(10);
    std_msgs::Float64MultiArray msg;

    int count = 0;

    while (ros::ok())
    {


        msg.data.push_back(count);

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}

