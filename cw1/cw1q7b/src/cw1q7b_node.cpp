#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include "Eigen/Dense"
#include <tf2_eigen/tf2_eigen.h>

//TODO: Complete the forward kinematic routine using Eigen. The function should compute the transformation between each
//frame and use tf broadcaster to publish the transformation.

double a[5] = {0.033, 0.155, 0.135, 0.0, 0.002};
double alpha[5] = {M_PI_2, 0, 0, M_PI_2, 0.0};
double d[5] = {0.145, 0.0, 0.0, 0.0, 0.185};
//double theta[5] = {0.0, M_PI_2 - 1.08, 0.0 - 2.49 , 3.0 * M_PI_2 - 1.82, 0.0};
double theta[5] = {170.0*M_PI/180.0, (65+90)*M_PI/180, (-146)*M_PI/180 , (-167.5)*M_PI/180, (167.5)*M_PI/180};


Eigen::Matrix4d fkine_standard(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d A;
    A(3, 3) = 1.0;
    A(3, 2) = 0.0;
    A(3, 1) = 0.0;
    A(3, 0) = 0.0;

    A(0, 0) = cos(theta);
    A(0, 1) = -sin(theta)*cos(alpha);
    A(0, 2) = sin(theta)*sin(alpha);
    A(0, 3) = a * cos(theta);

    A(1, 0) = sin(theta);
    A(1, 1) = cos(theta)*cos(alpha);
    A(1, 2) = -cos(theta)*sin(alpha);
    A(1, 3) = a * sin(theta);

    A(2, 0) = 0.0;
    A(2, 1) = sin(alpha);
    A(2, 2) = cos(alpha);
    A(2, 3) = d;

    return A;
}


void forward_kinematic(const sensor_msgs::JointState::ConstPtr& joint_msg, tf2_ros::TransformBroadcaster br){
    Eigen::Matrix4d A;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

    geometry_msgs::TransformStamped transform[5];
    

    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id = transform[4].header.frame_id  = "base_link";
    
    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = transform[4].header.stamp = ros::Time::now();
 

    transform[0].child_frame_id = "arm7b_link_1";
    transform[1].child_frame_id = "arm7b_link_2";
    transform[2].child_frame_id = "arm7b_link_3";
    transform[3].child_frame_id = "arm7b_link_4";
    transform[4].child_frame_id = "arm7b_link_5";


    for (int i = 0; i < 5; i++)
    {
        //the frame 0,1,2,3 are reversed, because in the '.xacro' file the z axes are reversed 
        if ((i == 0)|| (i == 1) ||(i == 2) ||(i == 3))
           A = fkine_standard(a[i], alpha[i], d[i], -joint_msg->position.at(i) + theta[i]);
        else
            A = fkine_standard(a[i], alpha[i], d[i], joint_msg->position.at(i) + theta[i]);

            T = T * A;

            Eigen::Affine3d T_affine;

            T_affine.matrix() = T;

            geometry_msgs::TransformStamped T_buffer = tf2::eigenToTransform(T_affine);

            transform[i].transform = T_buffer.transform;

            br.sendTransform(transform[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematic_node");
    ros::NodeHandle nh;

    //TODO: Initialise a subscriber to the topic "/joint_states" and its callback function forward_kinematic
    tf2_ros::TransformBroadcaster br;
    ros::Subscriber joint_sub_standard = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, boost::bind(forward_kinematic, _1, br));
    ros::spin();

}
