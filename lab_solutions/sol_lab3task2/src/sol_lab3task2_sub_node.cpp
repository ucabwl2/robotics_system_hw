#include "ros/ros.h"
//TODO: include the library for a msg for an array of doulbe
#include <sol_example_msg_srv/task_msg.h>

//TODO: Complete the callback function. Your callback function should rotate the input point and print the result out on the screen. Show the computational time on the screen.
void subscribeCallback(const sol_example_msg_srv::task_msg::ConstPtr& msg){
	
	double begin = ros::Time::now().toSec();;
	
	double x = msg->x.data; double y = msg->y.data; double z = msg->z.data;
    double qw = msg->qw.data; double qx = msg->qx.data;
    double qy = msg->qy.data; double qz = msg->qz.data;
    double new_x; double new_y; double new_z;

    //quaternion multiplication
    new_x = (1 - 2*pow(qy,2) - 2*pow(qz,2))*x + (2*qx*qy+2*qz*qw)*y + (2*qx*qz-2*qy*qw)*z;
    new_y = (2*qx*qy-2*qz*qw)*x + (1 - 2*pow(qx,2) - 2*pow(qz,2))*y + (2*qy*qz+2*qx*qw)*z;
    new_z = (2*qx*qz+2*qy*qw)*x + (2*qy*qz-2*qx*qw)*y + (1 - 2*pow(qx,2) - 2*pow(qy,2))*z;
    
    double end = ros::Time::now().toSec();;
    double time = end - begin;
    
    ROS_INFO("The computational time is %.6f seconds\n", time);
    ROS_INFO("The input point (x, y, z) = (%.2f, %.2f, %.2f)\n", x, y, z);
    ROS_INFO("The output point (x, y, z) = (%.2f, %.2f, %.2f)\n", new_x, new_y, new_z);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_point_subscriber");

    //TODO: Define your subscriber and link this with the callback function.
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/point_and_rot", 100, subscribeCallback);

    ros::spin();

    return 0;
}
