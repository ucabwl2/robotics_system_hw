#include "ros/ros.h"
//TODO: Include the library for an array of double message (Float64MultiArray)
#include <sol_example_msg_srv/task_msg.h>

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rotate_point_publisher");

    ros::NodeHandle nh;

    //TODO: Initialise the publisher and publishes to the topic "point_and_rot"   
    ros::Publisher pub = nh.advertise<sol_example_msg_srv::task_msg>("/point_and_rot", 100);

    ros::Rate loop_rate(10);

    //TODO: initialize your custom message
    sol_example_msg_srv::task_msg my_msg;

    while (nh.ok()){
	    //TODO: Generate a random point and a random quaternion. Make sure that the quaternion (qx, qy, qz, qw) is of unit norm.
	    //TODO: Generate a random point
		srand(time(0));
	    my_msg.x.data = fRand(-2.0, 2.0);
		srand(time(0));
		my_msg.y.data = fRand(-1.7, 1.7);
		srand(time(0));
		my_msg.z.data = fRand(-1.0, 1.8);

	    //TODO: Generate a random quaternion (make sure its norm is 1)
	    std::vector<double> quat, quat_unit;
	    
	    srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
	    srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
	    srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
	    srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
	    
	    double magnitute = sqrt(pow(quat[0],2) + pow(quat[1],2) + pow(quat[2],2) + pow(quat[3],2));
	    
	    quat_unit.push_back(quat[0] / magnitute); quat_unit.push_back(quat[1] / magnitute);
	    quat_unit.push_back(quat[2] / magnitute); quat_unit.push_back(quat[3] / magnitute);

	    my_msg.qw.data = quat_unit[0];
	    my_msg.qx.data = quat_unit[1];
	    my_msg.qy.data = quat_unit[2];
	    my_msg.qz.data = quat_unit[3];
	    
	    //TODO: Publish the message in the format of (px, py, pz, qx, qy, qz, qw)
	    pub.publish(my_msg);

	    ros::spinOnce();

	    loop_rate.sleep();
	}

    return 1;
}
