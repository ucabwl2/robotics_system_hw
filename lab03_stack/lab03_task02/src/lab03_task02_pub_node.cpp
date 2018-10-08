#include "ros/ros.h"
//TODO: Include the library for an array of double message (Float64MultiArray)

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rotate_point_publisher");

    ros::NodeHandle nh;
    //TODO: Initialise the publisher and publishes to the topic "point_and_rot"
    //TODO: Generate a random point and a random quaternion. Make sure that the quaternion (qx, qy, qz, qw) is of unit norm.
    //TODO: Publish the message in the format of (px, py, pz, qx, qy, qz, qw). 

    return 1;
}
