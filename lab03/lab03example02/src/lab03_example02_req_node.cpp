#include "ros/ros.h"
#include "example_msg_srv/test_srv.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rot_convert_client");
    ros::NodeHandle nh;
    ros::ServiceClient c = nh.serviceClient<example_msg_srv::test_srv>("rot_convert");

    example_msg_srv::test_srv srv;

    while (nh.ok())
    {
	srand(time(0));
        //srv.request.q.data = geometry_msgs::Quaternion(-0.228,-0.222,-0.555,0.768);
        srv.request.q.x = 0.0;
        srv.request.q.y = 0.7071068;
        srv.request.q.z= 0.7071068;
        srv.request.q.w = 0.0;
	//srand(time(0));
	//srv.request.y.data = fRand(-1.8, 1.7);
	//srand(time(0));
	//srv.request.z.data = fRand(-2.9, 1.6);
	if (c.call(srv))
        	ROS_INFO("The resulting RPY: (%.3f, %.3f, %.3f)\n", srv.response.x, srv.response.y, srv.response.z);

        usleep(10000);
    }

    return 123;
}
