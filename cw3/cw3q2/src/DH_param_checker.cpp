#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include "Eigen/Dense"
#include <tf2_eigen/tf2_eigen.h>

//TODO: Complete the forward kinematic routine using Eigen. The function should compute the transformation between each
//frame and use tf broadcaster to publish the transformation.

//    <arg name="trajectory_interface" default="false"/>
double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double alpha[7] = {-M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, 0.0};
double d[7] = {0.2025+0.1575, 0.0, 0.42, 0.0, 0.4,0.0,0.126};
double theta[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double pose1_values[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};


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
//void forward_kinematic(double pose1_values[], tf2_ros::TransformBroadcaster br){
    Eigen::Matrix4d A;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

    geometry_msgs::TransformStamped transform[7];
    

    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id = transform[4].header.frame_id  = transform[5].header.frame_id = transform[6].header.frame_id ="world";
    
    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = transform[4].header.stamp = transform[5].header.stamp = transform[6].header.stamp =ros::Time::now();
 

    transform[0].child_frame_id = "arm7b_link_1";
    transform[1].child_frame_id = "arm7b_link_2";
    transform[2].child_frame_id = "arm7b_link_3";
    transform[3].child_frame_id = "arm7b_link_4";
    transform[4].child_frame_id = "arm7b_link_5";
    transform[5].child_frame_id = "arm7b_link_6";
    transform[6].child_frame_id = "arm7b_link_7";


    for (int i = 0; i < 7; i++)
    {
        //the frame 0,1,2,3 are reversed, because in the '.xacro' file the z axes are reversed 
        //if ((i == 0)|| (i == 1) ||(i == 2) ||(i == 3))
        //   A = fkine_standard(a[i], alpha[i], d[i], -joint_msg->position.at(i) + theta[i]);
        //else
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
    ros::Subscriber joint_sub_standard = nh.subscribe<sensor_msgs::JointState>("/iiwa/joint_states", 5, boost::bind(forward_kinematic, _1, br));
    ros::spin();

}
