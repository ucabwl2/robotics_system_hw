#include "ros/ros.h"
#include "cw2q3/youbotKine.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"

#include "boost/foreach.hpp"

trajectory_msgs::JointTrajectoryPoint traj_pt;

//TODO: You can design your code to achieve the q4 task however you want as long as it stays in this file and it runs.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_cw2");

    youbot_kinematic youbot;

    youbot.init();

    int checkpoint_data = atoi(argv[1]);
    int dt = 0.01; //Maybe change here.

    if (checkpoint_data == 1)
    {
        //Load q4a data
    }
    else if (checkpoint_data == 2)
    {
        //Load q4b data
    }
    else if (checkpoint_data == 3)
    {
        //Load q4c data
    }

    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10);
    }

    return 123;
}
