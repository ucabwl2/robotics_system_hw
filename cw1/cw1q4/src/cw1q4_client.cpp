#include "ros/ros.h"
#include "cw1q4_srv/quat2zyx.h"
#include "cw1q4_srv/quat2rodrigues.h"
#include "cw1q4_srv/rotmat2quat.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rot_convert_client");
    ros::NodeHandle nh;
    ros::ServiceClient c = nh.serviceClient<cw1q4_srv::quat2zyx>("quatozyx");

    cw1q4_srv::quat2zyx srv;
    //tf::Quaternion q_test(0.0, 0.0, 0.0, 1.0);

    while (nh.ok())
    {
	srand(time(0));
        //srv.request.q.data = q_test;

	//srv.request.q.x = 0.0;
        //srv.request.q.y = pow(2,0.5)/2.0;
        //srv.request.q.z= 0.0;
        //srv.request.q.w = pow(2,0.5)/2.0;

        srv.request.q.x = 0.4811252;
        srv.request.q.y = 0.1924501;
        srv.request.q.z= 0.8552669;
        srv.request.q.w = 0.0;


        //srand(time(0));
	//srv.request.y.data = fRand(-1.8, 1.7);
	//srand(time(0));
	//srv.request.z.data = fRand(-2.9, 1.6);
	if (c.call(srv))
        	ROS_INFO("The resulting RzRyRx: (%.3f, %.3f, %.3f)\n", srv.response.z, srv.response.y, srv.response.x); 

        usleep(10000);
    }

    return 123;
}
