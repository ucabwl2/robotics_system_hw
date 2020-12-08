#include "ros/ros.h"
#include "example_msg_srv/test_srv.h"

bool convert_rot_quat(example_msg_srv::test_srv::Request &req, example_msg_srv::test_srv::Response &res)
{
    //double angle = sqrt(pow(req.x.data, 2) + pow(req.y.data, 2) + pow(req.z.data, 2));
    //double x = req.x.data/angle;
    //double y = req.y.data/angle;
    //double z = req.z.data/angle;
    //geometry_msgs::Quaternion q = req.q;

    double x = req.q.x;
    double y = req.q.y;
    double z = req.q.z;
    double w = req.q.w;

    res.x.data = atan2(x*z+w*y, -(y*z-w*x));
    res.y.data = acos(1-2*(pow(x,2)+pow(y,2)));
    res.z.data = atan2(x*z-w*y, y*z+w*x);

    //res.q.x = x * sin(angle/2);
    //res.q.y = y * sin(angle/2);
    //res.q.z = z * sin(angle/2);
    //res.q.w = cos(angle/2);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_rot_server");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("rot_convert", convert_rot_quat);

    ros::spin();
    return 12;
}
