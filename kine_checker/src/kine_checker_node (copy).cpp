#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include "cw2q4/youbotKine.h"

void youbot_kinematics() {

    Eigen::Matrix4d A;
    double curr_joint[5], pose1_values[5], pose2_values[5], pose3_values[5], pose4_values[5], pose5_values[5];

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    MatrixXd jacobian(6, 5);
    MatrixXd IK_closed;
    double* IK_ite;
    bool singularity;
	
	youbot_kinematic object;
	object.init();
	
	for (int i; i<5; i++)
		pose1_values[i] = 0.0;
	
	//feel free to change the values. but make sure they are inside the robot's workspace
	pose1_values[0] = -0.411883; pose1_values[1] = 1.30864; pose1_values[2] = -1.96052; pose1_values[3] = -1.07369; pose1_values[4] = -1.16671;
	pose2_values[0] = -1.66061; pose2_values[1] = 1.78931; pose2_values[2] = -2.54818; pose2_values[3] = -1.33999; pose2_values[4] = 1.70616;
    	pose3_values[0] = -0.268307;  pose3_values[1] = 1.48098; pose3_values[2] = -2.04278; pose3_values[3] = -0.859284; pose3_values[4] = -2.2123;
	pose4_values[0] = -1.5747; pose4_values[1] = 1.0566; pose4_values[2] = -1.99377; pose4_values[3] = -2.25452; pose4_values[4] = 0.975057;
	pose5_values[0] = 1.785; pose5_values[1] = 2.0513; pose5_values[2] = -2.54818; pose5_values[3] = -1.34124; pose5_values[4] = 0.472335;

	//JOINT1
	T = object.forward_kine(pose1_values, 5);
	jacobian = object.get_jacobian(pose1_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose1_values);
	singularity = object.check_singularity(pose1_values);
	std::cout << "///// JOINT 1 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose1_values[0] << ", " << pose1_values[1] << ", " << pose1_values[2] << ", " << pose1_values[3] << ", " << pose1_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

	//JOINT2
	T = object.forward_kine(pose2_values, 5);
	jacobian = object.get_jacobian(pose2_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose2_values);
	singularity = object.check_singularity(pose2_values);
	std::cout << "///// JOINT 2 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose2_values[0] << ", " << pose2_values[1] << ", " << pose2_values[2] << ", " << pose2_values[3] << ", " << pose2_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

	//JOINT3
	T = object.forward_kine(pose3_values, 5);
	jacobian = object.get_jacobian(pose3_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose3_values);
    singularity = object.check_singularity(pose3_values);
	std::cout << "///// JOINT 3 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose3_values[0] << ", " << pose3_values[1] << ", " << pose3_values[2] << ", " << pose3_values[3] << ", " << pose3_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

	//JOINT4
	T = object.forward_kine(pose4_values, 5);
	jacobian = object.get_jacobian(pose4_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose4_values);
    singularity = object.check_singularity(pose4_values);
	std::cout << "///// JOINT 4 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose4_values[0] << ", " << pose4_values[1] << ", " << pose4_values[2] << ", " << pose4_values[3] << ", " << pose4_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

	//JOINT5
	T = object.forward_kine(pose5_values, 5);
	jacobian = object.get_jacobian(pose5_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose5_values);
    singularity = object.check_singularity(pose5_values);
	std::cout << "///// JOINT 5 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose5_values[0] << ", " << pose5_values[1] << ", " << pose5_values[2] << ", " << pose5_values[3] << ", " << pose5_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity cheker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;
}

int main(int argc, char **argv)
{	
    ros::init(argc, argv, "kine_checker_node");
    ros::NodeHandle nh;

    youbot_kinematics();
    ros::spin();
}
