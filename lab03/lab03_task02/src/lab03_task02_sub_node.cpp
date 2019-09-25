#include "ros/ros.h"
//TODO: include the library for a msg for an array of doulbe

//TODO: Complete the callback function. Your callback function should rotate the input point and print the result out on the screen. Show the computational time on the screen.
void subscribeCallback();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_point_subscriber");

    //TODO: Define your subscriber and link this with the callback function.

    ros::spin();

    return 0;
}
