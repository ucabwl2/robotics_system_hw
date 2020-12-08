//TODO: Fill in the code. You will have to add your node in the launch file as well. The launch file is in cw3_launch.
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"                                                                
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include <cw3q2/iiwa14Kine.h>
#include <cmath>
#include "boost/foreach.hpp"

VectorXd mass(3),x(3),y(3),z(3), xy_position_matrix(6), z_position_matrix(3), external_torque(7),q3_external_torque(7);
int flag =0, row;
MatrixXd constant_matrix(6,3), third_row_constant_matrix(3,3);
// int time_counter=0;
VectorXd get_frame_joint_val(VectorXd joint_val, int frame)
{
    int th=frame;
    VectorXd v(frame);
    for (int i = 0; i < frame; i++)
        v(i) = joint_val(i);

    return v;
}

void callbackfunction(const sensor_msgs::JointState::ConstPtr& values)
{
	MatrixXd pose1(4,7);
	pose1 <<45*M_PI/180,-35*M_PI/180,55*M_PI/180,-30*M_PI/180,-25*M_PI/180,65*M_PI/180,-10*M_PI/180,
          5*M_PI/180,40*M_PI/180,80*M_PI/180,10*M_PI/180,8*M_PI/180,-50*M_PI/180,-10*M_PI/180,
	      -50*M_PI/180,60*M_PI/180,0*M_PI/180,-15*M_PI/180,60*M_PI/180,-25*M_PI/180,-50*M_PI/180,
	      0, -90*M_PI/180, -90*M_PI/180, 90*M_PI/180, -90*M_PI/180, 90*M_PI/180, -30*M_PI/180;

	VectorXd iiwa_position(7),iiwa_velocity(7),iiwa_effort(7), iiwa_accelerations(7), G(7), external_torque(7),v;
    Eigen::Matrix4d Timinus1 = Eigen::Matrix4d::Identity(4, 4);

    MatrixXd C(7,7), B(7,7), J(6,7),T(4,4);

    MatrixXd A = Eigen::MatrixXd::Constant(7, 3, 0), A_T;
    Vector3d z0,zi, p0, pi,result,p7;
    double z_estimated_1,z_estimated_2;
    
    
    z0(0) = 0;
    z0(1) = 0;
    z0(2) = 1;
    p0(0) = 0;
    p0(1) = 0;
    p0(2) = 0.1575;

    iiwa14_kinematic iiwa14;

    iiwa14.init();
	// std::cout << "values" << values << std::endl; 
	for (int i=0; i<7; i++)
  	 {
  	 				
  	 	
  	 	iiwa_position(i) = values->position.at(i);
  	 	// std::cout << "iiwa_position(i)" << iiwa_position(i) << std::endl; 
		iiwa_velocity(i) = values->velocity.at(i);
		iiwa_effort(i) = values->effort.at(i);
     }
     
     
     if((iiwa_position-(pose1.row(row)).transpose() ).norm()<0.1 )
     {
     		sleep(20);
     		// if ((iiwa_position-pose1).norm()<0.05)
     		// {

     	        if(row<3)
     	        {
     	        	G = iiwa14.getG(iiwa_position);
		     	
				external_torque = G - iiwa_effort;
				for(int i =0; i<7; i++)
				{
					v = get_frame_joint_val(iiwa_position,i+1);
					if(i==0)
			        {
			            zi = z0;
			            pi = p0;
			        }
			        else
			        {
			            Timinus1 = iiwa14.forward_kine(v, i);
			            for (int j = 0; j < 3; j++)
			            {
			                zi(j) = Timinus1(j,2);
			                pi(j) = Timinus1(j,3);
			            }
			        }
			        A.row(i) << -9.81*(-zi(1)),-9.81*zi(0), -9.81*(-zi(0)*pi(1)+zi(1)*pi(0));
		        }
		        result = ((A.transpose()*A)).inverse()*A.transpose()*external_torque;
		        mass(row) = result(2);
		        x(row) = result(0) / mass(row);
		        y(row) = result(1) / mass(row);
		        T = iiwa14.forward_kine(iiwa_position, 7);
		        p7 = T.block(0,3,3,1);

		        flag = 1;

	            constant_matrix.row(row*2) = T.block(0,0,1,3);
	            constant_matrix.row(row*2+1) = T.block(1,0,1,3);
	            xy_position_matrix(row*2) = x(row)-p7(0);
	            xy_position_matrix(row*2+1) = y(row)-p7(1);
	            z_position_matrix(row) = p7(2);


	            third_row_constant_matrix.row(row) = T.block(2,0,1,3);
	            // std::cout << "constant_matrix" << constant_matrix << std::endl; 
     	        }
     	        if(row==3)
     	        {
     	        	G = iiwa14.getG(iiwa_position);
					q3_external_torque = G - iiwa_effort;
					// std::cout << "q3_external_torque" << q3_external_torque<< std::endl; 
					flag = 1;
     	        }
     	        std::cout << "still running trajectory: " << (row+1) << " please wait" <<std::endl;
			    
     	        sleep(2);

	  }  
	     
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "iiwa14_traj_cw3b");

    iiwa14_kinematic iiwa14;

    iiwa14.init();

    int dt = 1; 
    double test;

    

	ros::NodeHandle nh;
	ros::Rate r(1000);
    //where do I publish below? 
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/object_iiwa/EffortJointInterface_trajectory_controller/command", 5);
    
    

	

	// rosbag::View view(mybag, rosbag::TopicQuery(topics));
	MatrixXd pose1(4,7);
	Vector3d result_;
	pose1 <<45*M_PI/180,-35*M_PI/180,55*M_PI/180,-30*M_PI/180,-25*M_PI/180,65*M_PI/180,-10*M_PI/180,
          5*M_PI/180,40*M_PI/180,80*M_PI/180,10*M_PI/180,8*M_PI/180,-50*M_PI/180,-10*M_PI/180,
	      -50*M_PI/180,60*M_PI/180,0*M_PI/180,-15*M_PI/180,60*M_PI/180,-25*M_PI/180,-50*M_PI/180,
	      0, -90*M_PI/180, -90*M_PI/180, 90*M_PI/180, -90*M_PI/180, 90*M_PI/180, -30*M_PI/180;
	
	for(int j=0; j<4; j++)
	{
		std::cout << "start to run trajectory: " << (j+1) << " please wait" <<std::endl;
		trajectory_msgs::JointTrajectory my_traj;
	    trajectory_msgs::JointTrajectoryPoint my_pt;
	    //Initialise all the necessary variables. It is important that the joint names match with the ones described in the urdf file of the robot.
		my_traj.header.stamp = ros::Time::now();
		my_traj.joint_names.push_back("object_iiwa_joint_1");
	    my_traj.joint_names.push_back("object_iiwa_joint_2");
	    my_traj.joint_names.push_back("object_iiwa_joint_3");
	    my_traj.joint_names.push_back("object_iiwa_joint_4");
	    my_traj.joint_names.push_back("object_iiwa_joint_5");
	    my_traj.joint_names.push_back("object_iiwa_joint_6");
	    my_traj.joint_names.push_back("object_iiwa_joint_7");
		my_pt.positions.resize(7);
		my_pt.velocities.resize(7);
	    my_pt.accelerations.resize(7);
	    int tfs = 20;

		row = j;
		my_pt.time_from_start.sec = tfs;
	   	for (int i = 0; i < 7;i++)
	        {
	            // std::cout << "i" << i << std::endl;
	   		    // std::cout << "i" << pose1(row,i) << std::endl; 
	   		    my_pt.positions.at(i) = pose1(row,i);
	        }
	        my_traj.points.push_back(my_pt);

	    sleep(5);

	    traj_pub.publish(my_traj);
	    ros::Subscriber sub_iiwa = nh.subscribe("/object_iiwa/joint_states", 10, callbackfunction);
	    
	    while(flag==0)
	    {
	    	ros::spinOnce();
	    }
	    //tfs = tfs; //+ 10;
	    flag = 0;
	}


    result_ = ((constant_matrix.transpose()*constant_matrix)).inverse()*constant_matrix.transpose()*xy_position_matrix;
    ROS_INFO("average_mass: %f", mass.sum()/3);
    ROS_INFO("x relative to frame 7: %f", result_(0));
    ROS_INFO("y relative to frame 7: %f", result_(1));
    ROS_INFO("z relative to frame 7: %f", result_(2));
    ROS_INFO("external_torque_1: %f", q3_external_torque(0));
    ROS_INFO("external_torque_2: %f", q3_external_torque(1));
    ROS_INFO("external_torque_3: %f", q3_external_torque(2));
    ROS_INFO("external_torque_4: %f", q3_external_torque(3));
    ROS_INFO("external_torque_5: %f", q3_external_torque(4));
    ROS_INFO("external_torque_6: %f", q3_external_torque(5));
    ROS_INFO("external_torque_7: %f", q3_external_torque(6));
    std::cout << "Task finished"<<std::endl;

}

//////////////////////
//////////////////////
//////////////////////
//////////////////////
//////////////////////
//////////////////////
//////////////////////
//////////////////////
//////////////////////




