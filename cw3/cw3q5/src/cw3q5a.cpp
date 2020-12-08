//TODO: Fill in the code. You will have to add your node in the launch file as well. The launch file is in cw3_launch.
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"                                                                
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include <cw3q2/iiwa14Kine.h>
#include <typeinfo>
#include "std_msgs/Float64.h"
#include "boost/foreach.hpp"



void callbackfunction(const sensor_msgs::JointState::ConstPtr& values)
{

	VectorXd iiwa_position(7),iiwa_velocity(7),iiwa_effort(7), iiwa_accelerations(7), G(7);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    MatrixXd C(7,7), B(7,7),J_cm(6,7), F_hard;

    iiwa14_kinematic iiwa14;
    double test;

    iiwa14.init();

    ros::NodeHandle nh;

    ros::Publisher acceleration_pub1 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration1", 1000);
    ros::Publisher acceleration_pub2 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration2", 1000);
    ros::Publisher acceleration_pub3 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration3", 1000);
    ros::Publisher acceleration_pub4 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration4", 1000);
    ros::Publisher acceleration_pub5 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration5", 1000);
    ros::Publisher acceleration_pub6 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration6", 1000);
    ros::Publisher acceleration_pub7 = nh.advertise<std_msgs::Float64>("/iiwa_acceleration7", 1000);
    ros::Publisher velocity_pub1 = nh.advertise<std_msgs::Float64>("/iiwa_velocity1", 1000);
	// std::cout << "values" << values << std::endl; 
	std_msgs::Float64 msg_a1 , msg_a2 ,msg_a3 ,msg_a4 ,msg_a5 ,msg_a6 ,msg_a7 ,msg_v1;
	for (int i=0; i<7; i++)
  	 {

  	 	
  	 	iiwa_position(i) = values->position.at(i);
  	 	// std::cout << "iiwa_position(i)" << iiwa_position(i) << std::endl; 
		iiwa_velocity(i) = values->velocity.at(i);
		iiwa_effort(i) = values->effort.at(i);
     }

     
     T = iiwa14.forward_kine(iiwa_position, 7);
     // std::cout << "T" << T << std::endl; 
     C = iiwa14.getC(iiwa_position, iiwa_velocity);
     G = iiwa14.getG(iiwa_position);
     B = iiwa14.getB(iiwa_position);
     iiwa_accelerations = B.inverse() *(iiwa_effort - G - C*iiwa_velocity);
     msg_a1.data = iiwa_accelerations(0);
     msg_a2.data = iiwa_accelerations(1);
     msg_a3.data = iiwa_accelerations(2);
     msg_a4.data = iiwa_accelerations(3);
     msg_a5.data = iiwa_accelerations(4);
     msg_a6.data = iiwa_accelerations(5);
     msg_a7.data = iiwa_accelerations(6);
     msg_v1.data = iiwa_velocity(0);
     // std::cout << "iiwa_accelerations" << iiwa_accelerations << std::endl; 
     // std::cout << typeid(iiwa_accelerations(0)).name() << std::endl; 
     // std::cout << typeid(test).name() << std::endl; 
     // J_cm = iiwa14.get_jacobian_cm(iiwa_position,7);

     // F_hard = (J_cm*B.inverse()*J_cm.transpose()).inverse()*( J_cm*B.inverse()*(iiwa_effort - G) );
     // std::cout << "F_hard"  << std::endl; 
     // std::cout << F_hard  << std::endl; 
     std::cout << "iiwa_accelerations"  << std::endl; 
     std::cout << iiwa_accelerations  << std::endl; 
     while (ros::ok())
    {
     acceleration_pub1.publish(msg_a1);
     acceleration_pub2.publish(msg_a2);
     acceleration_pub3.publish(msg_a3);
     acceleration_pub4.publish(msg_a4);
     acceleration_pub5.publish(msg_a5);
     acceleration_pub6.publish(msg_a6);
     acceleration_pub7.publish(msg_a7);
     velocity_pub1.publish(msg_v1);
     ros::spinOnce();
    }

}



int main (int argc, char **argv)
{
	ros::init(argc, argv, "iiwa14_traj_cw3");

    iiwa14_kinematic iiwa14;

    iiwa14.init();

    int dt = 1; 
    double test;

    rosbag::Bag mybag;

	//Specify the mode (Read/Write)
	//Check the definition of "MY_BAG_PATH" in the CMakeLists.txt
    mybag.open(MY_BAG_1, rosbag::bagmode::Read);

    // std::vector<std::string> topics;
    // topics.push_back(std::string("/iiwa/EffortJointInterface_trajectory_controller/command"));

    std::vector<std::string> topics;
    topics.push_back(std::string("/iiwa/EffortJointInterface_trajectory_controller/command"));


    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_pt;

	ros::NodeHandle nh;
	// ros::Rate r(1000);
    //where do I publish below? 
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/iiwa/EffortJointInterface_trajectory_controller/command", 5);
    
    //Initialise all the necessary variables. It is important that the joint names match with the ones described in the urdf file of the robot.
	my_traj.header.stamp = ros::Time::now();
	my_traj.joint_names.push_back("iiwa_joint_1");
    my_traj.joint_names.push_back("iiwa_joint_2");
    my_traj.joint_names.push_back("iiwa_joint_3");
    my_traj.joint_names.push_back("iiwa_joint_4");
    my_traj.joint_names.push_back("iiwa_joint_5");
    my_traj.joint_names.push_back("iiwa_joint_6");
    my_traj.joint_names.push_back("iiwa_joint_7");
	my_pt.positions.resize(7);
	my_pt.velocities.resize(7);
    my_pt.accelerations.resize(7);

	int tfs = 10;

	rosbag::View view(mybag, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            // sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
            trajectory_msgs::JointTrajectory::ConstPtr J = m.instantiate<trajectory_msgs::JointTrajectory>();
            std::cout << "i"<< std::endl;
            // std::cout << "J(0)"<< J->header.positions[0] <<std::endl;
            
            // if (J != NULL)
            // {
            	for(int k=0;k<3;k++)
                    {
            // std::cout << "i" << std::endl;
                    my_pt.time_from_start.sec = tfs;
		           	for (int i = 0; i < 7;i++)
		                {
		                    // std::cout << "i" << i << std::endl;
		           		    std::cout << "i" << J->points[k].positions[i] << std::endl; 
		           		    my_pt.positions.at(i) = J->points[k].positions[i];
		                }
		                my_traj.points.push_back(my_pt);
                        tfs = tfs + 10;
		            }
            // }


        }
    sleep(5);

    traj_pub.publish(my_traj);
    ros::Subscriber sub_iiwa = nh.subscribe("/iiwa/joint_states", 10, callbackfunction);
    
    

    // ros::spinOnce();
    ros::spin();

    mybag.close();

    return 0;
}

////////////////////////

////////////////////////

////////////////////////

////////////////////////

////////////////////////
