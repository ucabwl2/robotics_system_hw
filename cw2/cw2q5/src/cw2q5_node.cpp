#include "ros/ros.h"
#include "cw2q4/youbotKine.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"                                                                
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
//#include <Eigen/Dense>
//#include <youbotkdl_tester/YoubotKDL.h>

#include "boost/foreach.hpp"
//#include <youbotkdl_tester/YoubotKDL.h>

trajectory_msgs::JointTrajectoryPoint traj_pt;

//TODO: You can design your code to achieve the q4 task however you want as long as it stays in this file and it runs.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_cw2");

    youbot_kinematic youbot;

    youbot.init();

    int checkpoint_data = atoi(argv[1]);
    int dt = 1; //Maybe change here.

    if (checkpoint_data == 1)
    {
      

        //Load q4a data
        //ros::init(argc, argv, "lab7youbot_traj");

    	rosbag::Bag mybag;

    	//Specify the mode (Read/Write)
    	//Check the definition of "MY_BAG_PATH" in the CMakeLists.txt
        mybag.open(MY_BAG_A, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("joint_data"));

        trajectory_msgs::JointTrajectory my_traj;
        //trajectory_msgs::JointTrajectoryPoint my_pt;

   	ros::NodeHandle nh;

        ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

        //Initialise all the necessary variables. It is important that the joint names match with the ones described in the urdf file of the robot.
    	my_traj.header.stamp = ros::Time::now();
    	my_traj.joint_names.push_back("arm_joint_1");
    	my_traj.joint_names.push_back("arm_joint_2");
    	my_traj.joint_names.push_back("arm_joint_3");
    	my_traj.joint_names.push_back("arm_joint_4");
    	my_traj.joint_names.push_back("arm_joint_5");

    	traj_pt.positions.resize(50);
    	traj_pt.velocities.resize(50);

    	rosbag::View view(mybag, rosbag::TopicQuery(topics));

        int tfs = 1;
        MatrixXd  A(4,4); 
        A << 1,0,0,0,
          0,1,0,0,
          1,10,100,1000,
          0,1,20,300;            
        MatrixXd qs(1,5);
        qs(0,0) = 0.00122; qs(0,1) =0.39156; qs(0,2) =-0.35444; qs(0,3) =0.32441; qs(0,4) =0.01005;
        MatrixXd vs(1,5);
        vs(0,0) =0; vs(0,1) =0;vs(0,2) =0;vs(0,3) =0;vs(0,4) =0;
        MatrixXd qf(1,5);
        MatrixXd vf(1,5);
        MatrixXd B(4,5);
        MatrixXd C(4,1);
        
    	BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
                    if (J != NULL)
                    {
                        if (J->position.size() != 0)
                        {
                            
                            for (int k = 0; k < 5;k++)
                            {
                                C(0,0) = qs(0,k); C(1,0) = vs(0,k); C(2,0) = J->position.at(k); C(3,0) =  J->velocity.at(k);
                                B.col(k) = A.inverse() * C;
                                qf(0,k) = J->position.at(k);
                                vf(0,k) = J->velocity.at(k);
                            }
                            std::cout << "B" << B << std::endl;
                       
                            for (int i = 1; i < 11;i++)
                            {
                                traj_pt.time_from_start.sec = tfs;
                                for (int j = 0; j < 5;j++){
                                //std::cout << "i" << i << std::endl;
                                traj_pt.positions.at(j) = B(0,j) + B(1,j)*i + B(2,j)*pow(i,2) + B(3,j)*pow(i,3);
                                std::cout << "traj_pt.positions.at(j)" << traj_pt.positions.at(j) << std::endl;
                                //std::cout << J->position.at(i) << std::endl;
                                traj_pt.velocities.at(j) = B(1,j) + 2*B(2,j)*i + 3*B(3,j)*pow(i,2);
                                std::cout << "traj_pt.velocities.at(j)" << traj_pt.velocities.at(j) << std::endl;
                                //std::cout << J->velocity.at(i) << std::endl;
                                }
                                my_traj.points.push_back(traj_pt);
  
                            tfs = tfs + dt;
    
                            }
                            
                            
                            
                            
                        }
                    }
                    qs(0,0) = qf(0,0); qs(0,1) =qf(0,1); qs(0,2) =qf(0,2); qs(0,3) =qf(0,3); qs(0,4) =qf(0,4);
                    vs(0,0) =vf(0,0); vs(0,1) =vf(0,1);vs(0,2) =vf(0,2);vs(0,3) =vf(0,3);vs(0,4) =vf(0,4);

                }

    //This sleep varies in different machines. This is to prevent the publisher destroying itself before even publishes one trajectory.
    sleep(5);

    traj_pub.publish(my_traj);
    ros::spinOnce();

    mybag.close();
    return 5;

    
        
    }
///////////////////////////////////////////////////////////////////////////////////////////////
    else if (checkpoint_data == 2)
    {
        
        //Load q4a data
        //ros::init(argc, argv, "lab7youbot_traj");
     
    	rosbag::Bag mybag;

    	//Specify the mode (Read/Write)
    	//Check the definition of "MY_BAG_PATH" in the CMakeLists.txt
        mybag.open(MY_BAG_B, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("joint_data"));

        trajectory_msgs::JointTrajectory my_traj;
        //trajectory_msgs::JointTrajectoryPoint my_pt;

   	ros::NodeHandle nh;

        ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

        //Initialise all the necessary variables. It is important that the joint names match with the ones described in the urdf file of the robot.
    	my_traj.header.stamp = ros::Time::now();
    	my_traj.joint_names.push_back("arm_joint_1");
    	my_traj.joint_names.push_back("arm_joint_2");
    	my_traj.joint_names.push_back("arm_joint_3");
    	my_traj.joint_names.push_back("arm_joint_4");
    	my_traj.joint_names.push_back("arm_joint_5");

    	traj_pt.positions.resize(50);
    	traj_pt.velocities.resize(50);

    	rosbag::View view(mybag, rosbag::TopicQuery(topics));

        int tfs = 1;
        MatrixXd  A(4,4); 
        A << 1,0,0,0,
          0,1,0,0,
          1,10,100,1000,
          0,1,20,300;            
        MatrixXd qs(1,5);
        //qs(0,0) = 0.00122; qs(0,1) =0.39156; qs(0,2) =-0.35444; qs(0,3) =0.32441; qs(0,4) =0.01005;
        qs(0,0) = 0.001612; qs(0,1) =0.225594; qs(0,2) =-0.225; qs(0,3) =0.19187; qs(0,4) =0.008113;
        MatrixXd vs(1,5);
        vs(0,0) =0; vs(0,1) =0;vs(0,2) =0;vs(0,3) =0;vs(0,4) =0;
        MatrixXd qf(1,5);
        MatrixXd vf(1,5);
        MatrixXd B(4,5);
        MatrixXd C(4,1);
        //store one joints positions (theta) for each of the five cheker points
        double D[5];
        //store one joints velocities for each of the five cheker points
        double D2[5];
        //store five joints positions (theta) for each of the five cheker points
        MatrixXd G(5,5);
        //store five joints velocities for each of the five checker points
        MatrixXd G2(5,5);
        double start_q[5]={0.00122,0.39156,-0.35444,0.32441,0.01005};
        
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
        Eigen::Matrix4d start_T = Eigen::Matrix4d::Identity(4, 4);
        MatrixXd E(6,3);
        int index=0;
        //int index2=0;
        MatrixXd F(120,5);
        MatrixXd distance(1,120);
        double dist=0;
        //larger than the largest distance, so that it can be replaced when comparing
        double mini_dist=2.0;
        MatrixXd mini_order(1,5);
        int mini_h=0;

        int s=0;
 
         start_T = youbot.forward_kine_offset(start_q,5);
         std::cout << "start_T" << start_T << std::endl;
         E(5,0) = start_T(0,3);
         E(5,1) = start_T(1,3);
         E(5,2) = start_T(2,3);
         std::cout << "E.col(5)" << E.row(5) << std::endl;      
        // this for loop is to find out all the combination of all the order of the checker points
        for(int a=0; a<5; a++)
	{
            for(int b=0; b<5; b++)
	    {
                if(b!=a)
		{
		    for(int c=0; c<5; c++)
	            {
			if((c!=b) && (c!=a))
			{
		            for(int d=0; d<5; d++)
			    {
				if((d!=b) && (d!=a)&& (d!=c))
				{
				    for(int e=0; e<5; e++)
				    {
				        if((e!=b) && (e!=a) && (e!=c) && (e!=d))
					{
					    
					    F(s,0) = a;F(s,1) = b;F(s,2) = c;F(s,3) = d;F(s,4) = e;
                                            s +=1;
					}
				    }
				}
			    }
			}
		    }
		}			
            } 
	
	}

	std::cout << "F" << F << std::endl;


        // this for loop is to store the position and velocity information from the bag file, and store all the checker points positions. 
    	BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
                    if (J != NULL)
                    {
                        if (J->position.size() != 0)
                        {
                            
                            
                            for (int k = 0; k < 5;k++)
                            {
                             
                                D[k] = J->position.at(k);
                                D2[k] = J->velocity.at(k);
                            }
  
                            //std::cout << "B" << B << std::endl;
                            T = youbot.forward_kine_offset(D,5);
                            //std::cout << "T" << T << std::endl;
                            E(index,0) = T(0,3);
			    E(index,1) = T(1,3);
			    E(index,2) = T(2,3);
                            std::cout << "E.col(index)" << E.row(index) << std::endl;  
			    G(index,0) = D[0]; G(index,1) = D[1]; G(index,2) = D[2]; G(index,3) = D[3]; G(index,4) = D[4];     
                            G2(index,0) = D2[0]; G2(index,1) = D2[1]; G2(index,2) = D2[2]; G2(index,3) = D2[3]; G2(index,4) = D2[4];     
                        }
                    }
                    index +=1;

                }
        std::cout << "E" << E << std::endl;

        // this for loop is for finding the shortest distance from all the possible order, and store the order that can achieve the shortest distance
        for(int h=0; h<120; h++)
	{   
            int n =0;
            dist = 0;
            for(int m=0; m<5; m++)
            {   
                if(m==0)
                {
	            dist = pow(   pow(E(F(h,m),0) - E(5,0),2) + pow(  E(F(h,m),1)- E(5,1),2)  +  pow(E(F(h,m),2) - E(5,2),2)  ,0.5);
                    n = F(h,m);
                }
                else
                {
                    dist += pow(   pow(E(F(h,m),0) - E(n,0),2) + pow(  E(F(h,m),1)- E(n,1),2)  +  pow(E(F(h,m),2) - E(n,2),2)  ,0.5);
                    n = F(h,m);
                }
	    }	
            distance(0,h) = dist;
            if (dist <= mini_dist)
            {
		mini_dist = dist;
                mini_order(0,0) = F(h,0); mini_order(0,1) = F(h,1); mini_order(0,2) = F(h,2); mini_order(0,3) = F(h,3); mini_order(0,4) = F(h,4);  
                mini_h = h;  
 	    }
	}
        std::cout << "distance" << distance << std::endl;   
        std::cout << "mini_distance" << mini_dist << std::endl;   
        std::cout << "mini_order" << mini_order << std::endl;   
        std::cout << "mini_h" << mini_h << std::endl;   

        index=0;
        
        
        // this for loop is for publishing all the positions and velocities between each checker points
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
            if (J != NULL)
            {
                if (J->position.size() != 0)
                {
                    
                    
                    for (int k = 0; k < 5;k++)
                    {   
                        C(0,0) = qs(0,k); C(1,0) = vs(0,k); C(2,0) = G(mini_order(0,index),k); C(3,0) =  G2(mini_order(0,index),k);
                        B.col(k) = A.inverse() * C;
                        qf(0,k) =  G(mini_order(0,index),k);
                        vf(0,k) = G2(mini_order(0,index),k);
                        D[k] =  G(mini_order(0,index),k);
                    }

                    //std::cout << "B" << B << std::endl;
                    T = youbot.forward_kine_offset(D,5);
                    std::cout << "T" << T << std::endl;
                    E(index,0) = T(0,3);
		    E(index,1) = T(1,3);
		    E(index,2) = T(2,3);
                    std::cout << "E.col(index)" << E.row(index) << std::endl;

                    for (int i = 1; i < 11;i++)
                    {
                        traj_pt.time_from_start.sec = tfs;
                        for (int j = 0; j < 5;j++){
      
                        traj_pt.positions.at(j) = B(0,j) + B(1,j)*i + B(2,j)*pow(i,2) + B(3,j)*pow(i,3);
                        traj_pt.velocities.at(j) = B(1,j) + 2*B(2,j)*i + 3*B(3,j)*pow(i,2);
                        
                        }
                        my_traj.points.push_back(traj_pt);

                    tfs = tfs + dt;

                    }
                    
                    
                    
                    
                }
            }
            qs(0,0) = qf(0,0); qs(0,1) =qf(0,1); qs(0,2) =qf(0,2); qs(0,3) =qf(0,3); qs(0,4) =qf(0,4);
            vs(0,0) =vf(0,0); vs(0,1) =vf(0,1);vs(0,2) =vf(0,2);vs(0,3) =vf(0,3);vs(0,4) =vf(0,4);
            index +=1;

        }		

    //This sleep varies in different machines. This is to prevent the publisher destroying itself before even publishes one trajectory.
    sleep(5);

    traj_pub.publish(my_traj);
    ros::spinOnce();

    mybag.close();
  
   
    return 5;


    
        
    }
    else if (checkpoint_data == 3)
    {
        //Load q4c data
        //Load q4a data
        //ros::init(argc, argv, "lab7youbot_traj");

    	rosbag::Bag mybag;

    	//Specify the mode (Read/Write)
    	//Check the definition of "MY_BAG_PATH" in the CMakeLists.txt
        mybag.open(MY_BAG_C, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("target_position"));

        trajectory_msgs::JointTrajectory my_traj;
        //trajectory_msgs::JointTrajectoryPoint my_pt;

   	ros::NodeHandle nh;

        ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

        //Initialise all the necessary variables. It is important that the joint names match with the ones described in the urdf file of the robot.
    	my_traj.header.stamp = ros::Time::now();
    	my_traj.joint_names.push_back("arm_joint_1");
    	my_traj.joint_names.push_back("arm_joint_2");
    	my_traj.joint_names.push_back("arm_joint_3");
    	my_traj.joint_names.push_back("arm_joint_4");
    	my_traj.joint_names.push_back("arm_joint_5");

    	traj_pt.positions.resize(50);
    	traj_pt.velocities.resize(50);

    	rosbag::View view(mybag, rosbag::TopicQuery(topics));

        int tfs = 1;
        MatrixXd  A(4,4); 
        A << 1,0,0,0,
          0,1,0,0,
          1,10,100,1000,
          0,1,20,300;            
        MatrixXd qs(1,5);
        qs(0,0) = 0.00122; qs(0,1) =0.39156; qs(0,2) =-0.35444; qs(0,3) =0.32441; qs(0,4) =0.01005;
        MatrixXd vs(1,5);
        vs(0,0) =0; vs(0,1) =0;vs(0,2) =0;vs(0,3) =0;vs(0,4) =0;
        MatrixXd qf(1,5);
        MatrixXd vf(1,5);
        MatrixXd B(4,5);
        MatrixXd C(4,1);
        

    	BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
                    if (J != NULL)
                    {
                        if (J->position.size() != 0)
                        {
                            
                            for (int k = 0; k < 5;k++)
                            {
                                C(0,0) = qs(0,k); C(1,0) = vs(0,k); C(2,0) = J->position.at(k); C(3,0) =  J->velocity.at(k);
                                B.col(k) = A.inverse() * C;
                                qf(0,k) = J->position.at(k);
                                vf(0,k) = J->velocity.at(k);
                            }
                            std::cout << "B" << B << std::endl;
                       
                            for (int i = 1; i < 11;i++)
                            {
                                traj_pt.time_from_start.sec = tfs;
                                for (int j = 0; j < 5;j++){
                                //std::cout << "i" << i << std::endl;
                                traj_pt.positions.at(j) = B(0,j) + B(1,j)*i + B(2,j)*pow(i,2) + B(3,j)*pow(i,3);
                                std::cout << "traj_pt.positions.at(j)" << traj_pt.positions.at(j) << std::endl;
                                //std::cout << J->position.at(i) << std::endl;
                                traj_pt.velocities.at(j) = B(1,j) + 2*B(2,j)*i + 3*B(3,j)*pow(i,2);
                                std::cout << "traj_pt.velocities.at(j)" << traj_pt.velocities.at(j) << std::endl;
                                //std::cout << J->velocity.at(i) << std::endl;
                                }
                                my_traj.points.push_back(traj_pt);
  
                            tfs = tfs + dt;
    
                            }
                            
                            
                            
                            
                        }
                    }
                    qs(0,0) = qf(0,0); qs(0,1) =qf(0,1); qs(0,2) =qf(0,2); qs(0,3) =qf(0,3); qs(0,4) =qf(0,4);
                    vs(0,0) =vf(0,0); vs(0,1) =vf(0,1);vs(0,2) =vf(0,2);vs(0,3) =vf(0,3);vs(0,4) =vf(0,4);

                }

    //This sleep varies in different machines. This is to prevent the publisher destroying itself before even publishes one trajectory.
    sleep(5);

    traj_pub.publish(my_traj);
    ros::spinOnce();

    mybag.close();
    return 5;
        

    }

    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10);
    }

    return 123;
}
