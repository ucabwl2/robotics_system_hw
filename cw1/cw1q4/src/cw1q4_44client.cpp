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
    ros::ServiceClient c = nh.serviceClient<cw1q4_srv::rotmat2quat>("rotmattoquat");

    cw1q4_srv::rotmat2quat srv;
    //tf::Quaternion q_test(0.0, 0.0, 0.0, 1.0);

    while (nh.ok())
    {
	srand(time(0));
        //srv.request.q.data = q_test;
	srv.request.r1.data.resize(3);
        srv.request.r2.data.resize(3);
        srv.request.r3.data.resize(3);

        srv.request.r1.data[0] = -0.8987342;
        srv.request.r1.data[1] = 0.2531646;
        srv.request.r1.data[2] = 0.3580288;
        srv.request.r2.data[0] = 0.2531646;
        srv.request.r2.data[1] = -0.3670887;
        srv.request.r2.data[2] = 0.8950718;
        srv.request.r3.data[0] = 0.3580288;
        srv.request.r3.data[1] = 0.8950718;
        srv.request.r3.data[2] = 0.2658229;
        

        //srv.request.r1.data = [-1.0, 0.0, 0.0];
        //srv.request.r2.data = [0.0, 0.0, 1.0];
        //srv.request.r3.data = [0.0, 1.0 , 0.0];


        //srv.request.q.w = 0.0;
        //srand(time(0));
	//srv.request.y.data = fRand(-1.8, 1.7);
	//srand(time(0));
	//srv.request.z.data = fRand(-2.9, 1.6);
	if (c.call(srv))
        	ROS_INFO("The resulting Quaternion: (%.3f, %.3f, %.3f, %.3f)\n", srv.response.q.x, srv.response.q.y, srv.response.q.z, srv.response.q.w); 
        usleep(10000);
    }

    return 123;
}
