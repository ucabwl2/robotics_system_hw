#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include "cw3q2/iiwa14Kine.h"
#include "cw3q2/kdl_kine_solver.h"

void iiwa14_kinematics() {

    Eigen::Matrix4d A;
    //double curr_joint[5], pose1_values[5], pose2_values[5], pose3_values[5], pose4_values[5], pose5_values[5];
    VectorXd curr_joint(7), pose1_values(7), velocity_values(7);
    //VectorXd IK_iteration(7);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T_cm = Eigen::Matrix4d::Identity(4, 4);
    MatrixXd jacobian(7, 5);
    MatrixXd jacobian_cm(6,7);
    MatrixXd B(7,7), test_B(7,7),C;
    VectorXd G(7) ;
    int frame;
    MatrixXd IK_closed(8,7);
    VectorXd IK_ite;
    bool singularity;
    double psi;
	
	iiwa14_kinematic object;
	object.init();
	
	// for (int i; i<5; i++)
	// 	pose1_values[i] = 0.0;
	

	//feel free to change the values. but make sure they are inside the robot's workspace
	pose1_values << -0.411883,1.30864,-1.96052,-1.07369,-1.16671,-1.07369,-1.16671;
	// pose1_values << 90,90,90,90,90,90, 90;
	velocity_values << 2.0   ,2.0    ,2.0     ,2.0      ,2.0      ,2.0  ,    2.0   ;
	// pose1_values << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	//pose2_values[0] = -1.66061; pose2_values[1] = 1.78931; pose2_values[2] = -2.54818; pose2_values[3] = -1.33999; pose2_values[4] = 1.70616;
    	//pose3_values[0] = -0.268307;  pose3_values[1] = 1.48098; pose3_values[2] = -2.04278; pose3_values[3] = -0.859284; pose3_values[4] = -2.2123;
	//pose4_values[0] = -1.5747; pose4_values[1] = 1.0566; pose4_values[2] = -1.99377; pose4_values[3] = -2.25452; pose4_values[4] = 0.975057;
	//pose5_values[0] = 1.785; pose5_values[1] = 2.0513; pose5_values[2] = -2.54818; pose5_values[3] = -1.34124; pose5_values[4] = 0.472335;

	//JOINT1
	T = object.forward_kine(pose1_values, 7); 
	T_cm = object.forward_kine_cm(pose1_values, 7);
	jacobian = object.get_jacobian(pose1_values);
	jacobian_cm = object.get_jacobian_cm(pose1_values,7);

	IK_closed = object.inverse_kine_closed_form(T);
    curr_joint(0) = pose1_values(0); curr_joint(1) = pose1_values(1); curr_joint(2) = pose1_values(2); curr_joint(3) = pose1_values(3); curr_joint(4) = pose1_values(4); curr_joint(5) = pose1_values(5); curr_joint(6) = pose1_values(6);  
	IK_ite = object.inverse_kine_ite(T, curr_joint);
    psi = object.get_psi(T , pose1_values);
	B = object.getB(pose1_values);
	G = object.getG(pose1_values);
	C = object.getC(pose1_values, velocity_values);



//	singularity = object.check_singularity(pose1_values);
	std::cout << "///// JOINT 1 /////" << std::endl;
//	std::cout << "The input joint values are: \n" << "[" << pose1_values[0] << ", " << pose1_values[1] << ", " << pose1_values[2] << ", " << pose1_values[3] << ", " << pose1_values[4] << "]" << "\n" << std::endl;
	std::cout << "T \n"<< T << "\n" << std::endl;
    std::cout << "T_cm \n"<< T_cm << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "jacobian_cm: \n"<< jacobian_cm << "\n" << std::endl;
	std::cout << "IK_ite: \n"<< IK_ite << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
    
//	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
//      std::cout << "0 means no Singularity \n" << "\n" << std::endl;
    
    std::cout << "B (the same as KDL_B)\n"<< B << "\n" << std::endl;
    std::cout << "C \n"<< C << "\n" << std::endl;
    std::cout << "G (the same as KDL_G)\n"<< G << "\n" << std::endl;
    std::cout << "C*velocity (the same as KDL_C)\n"<< C*velocity_values << "\n" << std::endl;
    // std::cout << "psi \n"<< psi<< "\n" << std::endl;
	std::cout << "\n" << std::endl;

        /**
	//JOINT2
	T = object.forward_kine(pose2_values, 5);
	jacobian = object.get_jacobian(pose2_values);
	IK_closed = object.inverse_kine_closed_form(T);
        curr_joint[0] = pose2_values[0]; curr_joint[1] = pose2_values[1]; curr_joint[2] = pose2_values[2]; curr_joint[3] = pose2_values[3]; curr_joint[4] = pose2_values[4];
	IK_ite = object.inverse_kine_ite(T, curr_joint);
	//singularity = object.check_singularity(pose2_values);
        //std::cout << "rotatioin matrix T: \n"<< T << "\n" << std::endl;
	std::cout << "///// JOINT 2 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose2_values[0] << ", " << pose2_values[1] << ", " << pose2_values[2] << ", " << pose2_values[3] << ", " << pose2_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	//std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
        for(int k=0; k<5; k++)
	{
	    IK_iteration[k] = *IK_ite;
            IK_ite++;
	}
	std::cout << "Iterative form IK: \n"<< "[" << IK_iteration[0] << ", " << IK_iteration[1] << ", " << IK_iteration[2] << ", " <<  IK_iteration[3] <<  ", " << IK_iteration[4] << "]" << "\n"<< std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
        std::cout << "0 means no Singularity \n" << "\n" << std::endl;
	std::cout << "\n" << std::endl;
        
        
	//JOINT3
	T = object.forward_kine(pose3_values, 5);
	jacobian = object.get_jacobian(pose3_values);
	IK_closed = object.inverse_kine_closed_form(T);
        curr_joint[0] = pose3_values[0]; curr_joint[1] = pose3_values[1]; curr_joint[2] = pose3_values[2]; curr_joint[3] = pose3_values[3]; curr_joint[4] = pose3_values[4];
	IK_ite = object.inverse_kine_ite(T, curr_joint);
    singularity = object.check_singularity(pose3_values);
	std::cout << "///// JOINT 3 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose3_values[0] << ", " << pose3_values[1] << ", " << pose3_values[2] << ", " << pose3_values[3] << ", " << pose3_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	//std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
        for(int k=0; k<5; k++)
	{
	    IK_iteration[k] = *IK_ite;
            IK_ite++;
	}
	std::cout << "Iterative form IK: \n"<< "[" << IK_iteration[0] << ", " << IK_iteration[1] << ", " << IK_iteration[2] << ", " <<  IK_iteration[3] <<  ", " << IK_iteration[4] << "]" << "\n"<< std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
        std::cout << "0 means no Singularity \n" << "\n" << std::endl;
	std::cout << "\n" << std::endl;

	//JOINT4
	T = object.forward_kine(pose4_values, 5);
	jacobian = object.get_jacobian(pose4_values);
	IK_closed = object.inverse_kine_closed_form(T);
        curr_joint[0] = pose4_values[0]; curr_joint[1] = pose4_values[1]; curr_joint[2] = pose4_values[2]; curr_joint[3] = pose4_values[3]; curr_joint[4] = pose4_values[4];
	IK_ite = object.inverse_kine_ite(T, curr_joint);
    singularity = object.check_singularity(pose4_values);
	std::cout << "///// JOINT 4 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose4_values[0] << ", " << pose4_values[1] << ", " << pose4_values[2] << ", " << pose4_values[3] << ", " << pose4_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	//std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
        for(int k=0; k<5; k++)
	{
	    IK_iteration[k] = *IK_ite;
            IK_ite++;
	}
	std::cout << "Iterative form IK: \n"<< "[" << IK_iteration[0] << ", " << IK_iteration[1] << ", " << IK_iteration[2] << ", " <<  IK_iteration[3] <<  ", " << IK_iteration[4] << "]" << "\n"<< std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
        std::cout << "0 means no Singularity \n" << "\n" << std::endl;
	std::cout << "\n" << std::endl;

	//JOINT5
	T = object.forward_kine(pose5_values, 5);
	jacobian = object.get_jacobian(pose5_values);
	IK_closed = object.inverse_kine_closed_form(T);
        curr_joint[0] = pose5_values[0]; curr_joint[1] = pose5_values[1]; curr_joint[2] = pose5_values[2]; curr_joint[3] = pose5_values[3]; curr_joint[4] = pose5_values[4];
	IK_ite = object.inverse_kine_ite(T, curr_joint);
    singularity = object.check_singularity(pose5_values);
	std::cout << "///// JOINT 5 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose5_values[0] << ", " << pose5_values[1] << ", " << pose5_values[2] << ", " << pose5_values[3] << ", " << pose5_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	//std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
        for(int k=0; k<5; k++)
	{
	    IK_iteration[k] = *IK_ite;
            IK_ite++;
	}
	std::cout << "Iterative form IK: \n"<< "[" << IK_iteration[0] << ", " << IK_iteration[1] << ", " << IK_iteration[2] << ", " <<  IK_iteration[3] <<  ", " << IK_iteration[4] << "]" << "\n"<< std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
        std::cout << "0 means no Singularity \n" << "\n" << std::endl;
	std::cout << "\n" << std::endl;
        */
       
}

void robot_kinematics() {

    Eigen::Matrix4d A;
    //double curr_joint[5], pose1_values[5], pose2_values[5], pose3_values[5], pose4_values[5], pose5_values[5];
    VectorXd curr_joint(7), pose1_values(7),velocity_values(7);
    //VectorXd IK_iteration(7);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T_cm = Eigen::Matrix4d::Identity(4, 4);
    MatrixXd jacobian(7, 5);
    MatrixXd jacobian_cm(6,7);
    MatrixXd test_B;
    VectorXd test_G,test_C;
    int frame;
    MatrixXd IK_closed(8,7);
    VectorXd IK_ite;
    bool singularity;
    double psi;
	
	robot_kinematic object;
	object.init();

	pose1_values << -0.411883,1.30864,-1.96052,-1.07369,-1.16671,-1.07369,-1.16671;
	// pose1_values << 90,90,90,90,90,90, 90;
	velocity_values << 2.0   ,2.0    ,2.0     ,2.0      ,2.0      ,2.0  ,    2.0   ;


    test_B = object.getB(pose1_values);
    std::cout << "KDL_B" << "\n" << std::endl;
    std::cout << test_B  << "\n" << std::endl;
    test_G = object.getG(pose1_values);
    std::cout << "KDL_G" << "\n" << std::endl;
    std::cout << test_G  << "\n" << std::endl;

    test_C = object.getC(pose1_values,velocity_values);
    std::cout << "KDL_C" << "\n" << std::endl;
    std::cout << test_C  << "\n" << std::endl;
}
int main(int argc, char **argv)
{	
    ros::init(argc, argv, "iiwa_kine_checker_node");
    ros::NodeHandle nh;

    iiwa14_kinematics();
    robot_kinematics();
    ros::spin();
}
