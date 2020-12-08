#include "ros/ros.h"
//TODO: Include the header file for the three services
#include "cw1q4_srv/quat2zyx.h"
#include "cw1q4_srv/quat2rodrigues.h"
#include "cw1q4_srv/rotmat2quat.h"
//TODO: Complete these three functions
bool convert_quat2zyx(cw1q4_srv::quat2zyx::Request &req, cw1q4_srv::quat2zyx::Response &res ){

    double x = req.q.x;
    double y = req.q.y;
    double z = req.q.z;
    double w = req.q.w;

    res.x.data = atan2(2*(w*x+y*z),1-2*(pow(x,2)+pow(y,2)));
    if (2*(w*y - z*x)>=1){
        res.y.data = copysign(M_PI / 2, 2*(w*y - z*x));
    }else{
        res.y.data = asin(2*(w*y - z*x));
    }
    res.z.data = atan2(2*(w*z+x*y),1-2*(pow(y,2)+pow(z,2)));
    
    
    return true;
}
bool convert_quat2rodrigues(cw1q4_srv::quat2rodrigues::Request &req, cw1q4_srv::quat2rodrigues::Response &res){

    double ux;
    double uy;
    double uz;

    double theta = 2*acos(req.q.w);
    if (theta != 0){
        ux = req.q.x / sin(theta/2);
        uy = req.q.y / sin(theta/2);
        uz = req.q.z / sin(theta/2);
    }
    else{
        ux = 0;
        uy = 0;
        uz = 0;
    }
    double X = ux * theta;
    double Y = uy * theta;
    double Z = uz * theta;
    
    res.x.data = X;
    res.y.data = Y;
    res.z.data = Z;
    
 
}
bool convert_rotmat2quat(cw1q4_srv::rotmat2quat::Request &req, cw1q4_srv::rotmat2quat::Response &res){
    std_msgs::Float64MultiArray a;
    std_msgs::Float64MultiArray b;
    std_msgs::Float64MultiArray c;
    double w_square;
    double roll_square;
    double pitch_square;
    double a11, a12, a13, a21, a22, a23, a31, a32, a33;
    
    a11 = req.r1.data[0];
    a12 = req.r1.data[1];
    a13 = req.r1.data[2];
    a21 = req.r2.data[0];
    a22 = req.r2.data[1];
    a23 = req.r2.data[2];
    a31 = req.r3.data[0];
    a32 = req.r3.data[1];
    a33 = req.r3.data[2];

    float Trace = a11 + a22 + a33;
    if (Trace > 0){
        float S = pow(Trace+1.0,0.5)*2;
        res.q.w = 0.25*S;
        res.q.x = (a32 - a23) / S;
        res.q.y = (a13 - a31) / S;
        res.q.z = (a21 - a12) / S;
    } else if ((a11 > a22)&(a11 > a33)){
        float S = pow(1.0 + a11 - a22 - a33,0.5)*2;
        res.q.w = (a32 - a23) / S;
        res.q.x = 0.25*S;
        res.q.y = (a12 + a21) / S;
        res.q.z = (a13 + a31) /S;
    } else if (a22 > a33){
        float S = pow(1 + a22 - a11 - a33,0.5) * 2;
        res.q.w = (a13 - a31) / S;
        res.q.x = (a12 + a21) / S;
        res.q.y = 0.25*S;
        res.q.z = (a23 + a32) / S;
    } else {
        float S =pow(1.0 + a33 - a11 - a22,0.5) * 2;
        res.q.w = (a21 - a12) /S;
        res.q.x = (a13 + a31)/S;
        res.q.y = (a23 + a32)/S;
        res.q.z = 0.25*S;
}
     //I learn this from:https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_converter");
    ros::NodeHandle nh;

    //TODO: Define three services
    ros::ServiceServer server_4_2 = nh.advertiseService("quatozyx", convert_quat2zyx);
    ros::ServiceServer server_4_4 = nh.advertiseService("rotmattoquat", convert_rotmat2quat);
    ros::ServiceServer server_4_3 = nh.advertiseService("quattorodrigues", convert_quat2rodrigues);
    

    ros::spin();
    return 5;
}
