#ifndef YOUBOTKDL_TESTER_YOUBOTKDL_H
#define YOUBOTKDL_TESTER_YOUBOTKDL_H

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

class YoubotKDL
{

protected:
    ros::NodeHandle n;
    ros::Publisher traj_publisher;
    ros::Subscriber subscriber_joint_state;
    tf2_ros::TransformBroadcaster pose_broadcaster;
    double DH_params[5][5];
    double joint_limit_min[5];
    double joint_limit_max[5];
    double joint_offset[5];

public:
    KDL::Frame current_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position;

    void init();
    void broadcast_pose(KDL::Frame current_pose);
    void setup_kdl_chain();
    KDL::Jacobian get_jacobian(KDL::JntArray current_joint_position);
    KDL::JntArray inverse_kinematics_closed(KDL::Frame desired_pose);
    KDL::Frame forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    void publish_joint_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory);

};

#endif //INVERSE_KINEMATICS_YOUBOTKDL_H
