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
    ros::ServiceClient c = nh.serviceClient<cw1q4_srv::quat2rodrigues>("quattorodrigues");

    cw1q4_srv::quat2rodrigues srv;
    //tf::Quaternion q_test(0.0, 0.0, 0.0, 1.0);

    while (nh.ok())
    {
	srand(time(0));
        
	srv.request.q.x = 0.2250176;
        srv.request.q.y = 0.5625439;
        srv.request.q.z= 0.7955573;
        srv.request.q.w = 0.0;

        //srv.request.q.x = 0.0;
        //srv.request.q.y = 0.0;
        //srv.request.q.z= 0.0;
        //srv.request.q.w = 1.0;

        
	if (c.call(srv))
        	ROS_INFO("The resulting rodrigues XYZ: (%.3f, %.3f, %.3f)\n", srv.response.x, srv.response.y, srv.response.z); 

        usleep(10000);
    }

    return 123;
}
