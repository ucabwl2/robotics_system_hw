#include <cw2q4/youbotKine.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>


/*youbotKine::youbot_kinematic(ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}
*/
void youbot_kinematic::init()
{   
    //a
    DH_params[0][0] = -0.033;   DH_params[1][0] = 0.155;  DH_params[2][0] = 0.135; DH_params[3][0] = -0.002;  DH_params[4][0] = 0.0;
    //alpha
    DH_params[0][1] = M_PI_2;   DH_params[1][1] = 0.0;    DH_params[2][1] = 0.0;   DH_params[3][1] = M_PI_2;  DH_params[4][1] = M_PI;
    //d
    DH_params[0][2] = 0.145;    DH_params[1][2] = 0.0;    DH_params[2][2] = 0.0;   DH_params[3][2] = 0.0;     DH_params[4][2] = -0.185;
    //theta
    DH_params[0][3] = M_PI;     DH_params[1][3] = M_PI_2; DH_params[2][3] = 0.0;   DH_params[3][3] = -M_PI_2; DH_params[4][3] = M_PI;

    joint_offset[0] = 170*M_PI/180;
    joint_offset[1] = 65*M_PI/180;
    joint_offset[2] = -146*M_PI/180;
    joint_offset[3] = 102.5*M_PI/180;
    joint_offset[4] = 167.5*M_PI/180;

    joint_limit_min[0] = -169*M_PI/180;
    joint_limit_min[1] = -65*M_PI/180;
    joint_limit_min[2] = -150*M_PI/180;
    joint_limit_min[3] = -102.5*M_PI/180;
    joint_limit_min[4] = -167.5*M_PI/180;

    joint_limit_max[0] = 169*M_PI/180;
    joint_limit_max[1] = 90*M_PI/180;
    joint_limit_max[2] = 146*M_PI/180;
    joint_limit_max[3] = 102.5*M_PI/180;
    joint_limit_max[4] = 167.5*M_PI/180;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &youbot_kinematic::joint_state_callback, this);
}


void youbot_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for(int i = 0; i < 5; i++)
        current_joint_position[i] = q->position.at(i);

    current_pose = forward_kine(current_joint_position, 5);
    broadcast_pose(current_pose);
}

Matrix4d youbot_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Matrix4d A;
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


Matrix4d youbot_kinematic::forward_kine(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_val[i] + DH_params[i][3]);

        T = T * A;
    }

    return T;
}
///////////////////
//////////////////
/////////////////
Matrix4d youbot_kinematic::forward_kine_offset(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        if (i == 0)
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_offset[i] - (joint_val[i] + DH_params[i][3]));
        else
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], (joint_val[i] + DH_params[i][3]) - joint_offset[i]);

        T = T * A;
    }

    return T;
}

void youbot_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "base_link";
    T.child_frame_id = "arm_end_effector";

    pose_br.sendTransform(T);
}

MatrixXd youbot_kinematic::get_jacobian(double joint_val[])
{
    //TODO: Fill in this function to complete the question 3a

    Eigen::Matrix4d T11 = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T12 = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T13 = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T14 = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T15 = Eigen::Matrix4d::Identity(4, 4);
    double J11[1];
    double J12[2];
    double J13[3];
    double J14[4];
    double J15[5];
    J11[0] = joint_val[0];
    //std::cout << "J1[0]: \n"<< J1[0] << "\n" << std::endl;
    T11 = youbot_kinematic::forward_kine(J11, 1);
    //std::cout << "T11: \n"<< T11 << "\n" << std::endl;

    J12[0] = joint_val[0];
    J12[1] = joint_val[1];
    //std::cout << "J12[0]: \n"<< J12[0] << "\n" << "J12[1]: \n"<< J12[1] << "\n" << std::endl;
    T12 = youbot_kinematic::forward_kine(J12, 2);
    //std::cout << "xT12: \n"<< T12 << "\n" << std::endl;

    J13[0] = joint_val[0];
    J13[1] = joint_val[1];
    J13[2] = joint_val[2];
    T13 = youbot_kinematic::forward_kine(J13, 3);
    //std::cout << "xT123: \n"<< T123 << "\n" << std::endl;

    J14[0] = joint_val[0];
    J14[1] = joint_val[1];
    J14[2] = joint_val[2];
    J14[3] = joint_val[3];
    T14 = youbot_kinematic::forward_kine(J14, 4);
    //std::cout << "xT1234: \n"<< T1234 << "\n" << std::endl;

    J15[0] = joint_val[0];
    J15[1] = joint_val[1];
    J15[2] = joint_val[2];
    J15[3] = joint_val[3];
    J15[4] = joint_val[4];
    T15 = youbot_kinematic::forward_kine(J15, 5);
    //std::cout << "xT12345: \n"<< T12345 << "\n" << std::endl;

    //double Jp1[3],Jp2[3],Jp3[3],Jp4[3],Jp5[3];
    MatrixXd ja(6,5);

    ja(0,0) =  -T15(1,3); //T15(2,3)*T15(1,2) - T15(1,3)*T15(2,2);
    ja(1,0) =  T15(0,3);  //T15(2,2)*T15(0,3) - T15(2,3)*T15(0,2);
    ja(2,0) =  0;         //J;T15(1,3)*T15(0,2) - T15(0,3)*T15(1,2);
    ja(3,0) = 0;
    ja(4,0) = 0;
    ja(5,0) = 1;
    ja(0,1) = T11(1,2)*(T15(2,3)-T11(2,3)) - T11(2,2)*(T15(1,3)-T11(1,3));
    ja(1,1) = T11(2,2)*(T15(0,3)-T11(0,3)) - T11(0,2)*(T15(2,3)-T11(2,3));
    ja(2,1) = T11(0,2)*(T15(1,3)-T11(1,3)) - T11(1,2)*(T15(0,3)-T11(0,3));
    ja(3,1) = T11(0,2);
    ja(4,1) = T11(1,2);
    ja(5,1) = T11(2,2);
    ja(0,2) = T12(1,2)*(T15(2,3)-T12(2,3)) - T12(2,2)*(T15(1,3)-T12(1,3));
    ja(1,2) = T12(2,2)*(T15(0,3)-T12(0,3)) - T12(0,2)*(T15(2,3)-T12(2,3));
    ja(2,2) = T12(0,2)*(T15(1,3)-T12(1,3)) - T12(1,2)*(T15(0,3)-T12(0,3));
    ja(3,2) = T12(0,2);
    ja(4,2) = T12(1,2);
    ja(5,2) = T12(2,2);
    ja(0,3) = T13(1,2)*(T15(2,3)-T13(2,3)) - T13(2,2)*(T15(1,3)-T13(1,3));
    ja(1,3) = T13(2,2)*(T15(0,3)-T13(0,3)) - T13(0,2)*(T15(2,3)-T13(2,3));
    ja(2,3) = T13(0,2)*(T15(1,3)-T13(1,3)) - T13(1,2)*(T15(0,3)-T13(0,3));
    ja(3,3) = T13(0,2);
    ja(4,3) = T13(1,2);
    ja(5,3) = T13(2,2);
    ja(0,4) = T14(1,2)*(T15(2,3)-T14(2,3)) - T14(2,2)*(T15(1,3)-T14(1,3));
    ja(1,4) = T14(2,2)*(T15(0,3)-T14(0,3)) - T14(0,2)*(T15(2,3)-T14(2,3));
    ja(2,4) = T14(0,2)*(T15(1,3)-T14(1,3)) - T14(1,2)*(T15(0,3)-T14(0,3));
    ja(3,4) = T14(0,2);
    ja(4,4) = T14(1,2);
    ja(5,4) = T14(2,2);
    return ja;
    
}

MatrixXd youbot_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //TODO: Fill in this function to complete the question 3c
    MatrixXd inv(4,6);
    double theta1,theta2_1,theta2_2,theta3,theta4_1,theta4_2,theta5;
    double a1=DH_params[0][0],a2 = DH_params[1][0], a3 = DH_params[2][0],a4=DH_params[3][0];
    double d5 = DH_params[4][2], d1 = DH_params[0][2];
    double x=pose(0,3),y=pose(1,3),z=pose(2,3);
    double A1,A2,B1,B2,Z1,Z2,Z3_1,Z4_1,Z3_2,Z4_2;
    double r11=pose(0,0),r12=pose(0,1),r13=pose(0,2),r21=pose(1,0),r22=pose(1,1),r23=pose(1,2),r31=pose(2,0),r32=pose(2,1),r33=pose(2,2);

    
    //Solve for theta1
    theta1 = atan2(r23,r13);
    inv(0, 0) = inv(1, 0) = theta1;

    //Check with the joint limit for the other solution.
    if (inv(0, 0) <= 0.0)
        inv(2, 0) = inv(3, 0) = theta1 + M_PI;
    else
        inv(2, 0) = inv(3, 0) = theta1 - M_PI;

    
    //Solve for theta3
    for (int i = 0; i < 2; i++)
    {
        
        //Rename the variables so that the code is more readable.
        double th1 = inv(2*i, 0);
        //std::cout << "th1\n"<< th1 << "\n" << std::endl;

        if (sin(th1) == 0)
            Z1 = x/cos(th1) + a4*r33 + d5*r13/cos(th1) + a1;
        else
            Z1 = y/sin(th1) + a4*r33 + d5*r23/sin(th1) + a1;


        if (sin(th1) == 0)
            Z2 = z - d1 - a4*r13/cos(th1) + d5*r33;
        else
            Z2 = z - d1 - a4*r23/sin(th1) + d5*r33;


    theta3 = acos((pow(Z1,2)+pow(Z2,2)-pow(a2,2)-pow(a3,2)) / (2*a2*a3)); 
    inv(2*i, 2) = theta3;
    inv(2*i + 1, 2) = -theta3;

    //Rename the variables so that the code is more readable.
    double th3_1 = inv(2*i, 2);
    double th3_2 = inv(2*i + 1, 2);


    B1 = a3*sin(th3_1);
    B2 = a3*sin(th3_2);

    A1 = a3*cos(th3_1) + a2;
    A2 = a3*cos(th3_2) + a2;

    //Rename the variables so that the code is more readable.
    double gamma_1 = atan2(B1, A1);
    double gamma_2 = atan2(B2, A2);

    theta2_1 = atan2(Z1,Z2) - gamma_1;
    theta2_2 = atan2(Z1,Z2) - gamma_2;

    inv(2*i, 1) = theta2_1;
    inv(2*i + 1, 1) = theta2_2;

    
    //solve for theta4
    inv(2*i, 3) =  atan2(r13/cos(th1),r33) - theta2_1 - th3_1;
    inv(2*i + 1, 3) =  atan2(r13/cos(th1),r33) - theta2_2 - th3_2;
    
    //Solve for theta5
    double th2;
    double th3;
    double th4;
    double th234_1;
    double th234_2;

    th234_1 = inv(2*i,1) + inv(2*i,2) + inv(2*i,3);
    th234_2 = inv(2*i+1,1) + inv(2*i+1,2) + inv(2*i+1,3);
    inv(2*i, 4) = atan2(r32/(-sin(th234_1)),r31/(-sin(th234_1)));
    inv(2*i+1, 4) = atan2(r32/(-sin(th234_2)),r31/(-sin(th234_2)));


    }


    //Check if the inverse kinematic solutions are in the robot workspace
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            if ((inv(i, j) <= joint_limit_min[j]) || (inv(i, j) >= joint_limit_max[j]))
            {
                //If the solution is very close to the limit, it could be because of the numerical error.
                if (std::abs(inv(i, j) - joint_limit_min[j]) < 1.5*M_PI/180)
                {
                    inv(i, j) = joint_limit_min[j];
                }
                else if (std::abs(inv(i, j) - joint_limit_max[j]) < 1.5*M_PI/180)
                {
                    inv(i, j) = joint_limit_max[j];
                }
                else
                {
                    //Put some value there to let you know that this solution is not valid.
                    inv(i, 5) = -j - 1;
                    break;
                }
            }
            else
            {
                //Put some value there to let you know that this solution is valid.
                inv(i, 5) = 0.0;
            }
        }
    }

    return inv;
}

MatrixXd youbot_kinematic::convert_rotmat2quat(Matrix3d rotmat)
{
    
    double w_square;
    MatrixXd quat(1,4);
    double roll_square;
    double pitch_square;
    double a11, a12, a13, a21, a22, a23, a31, a32, a33;
    
    a11 = rotmat(0,0);
    a12 = rotmat(0,1);
    a13 = rotmat(0,2);
    a21 = rotmat(1,0);
    a22 = rotmat(1,1);
    a23 = rotmat(1,2);
    a31 = rotmat(2,0);
    a32 = rotmat(2,1);
    a33 = rotmat(2,2);

    float Trace = a11 + a22 + a33;
    if (Trace > 0){
        float S = pow(Trace+1.0,0.5)*2;
        quat(0,3) = 0.25*S;
        quat(0,0) = (a32 - a23) / S;
        quat(0,1) = (a13 - a31) / S;
        quat(0,2) = (a21 - a12) / S;
    } else if ((a11 > a22)&(a11 > a33)){
        float S = pow(1.0 + a11 - a22 - a33,0.5)*2;
        quat(0,3) = (a32 - a23) / S;
        quat(0,0) = 0.25*S;
        quat(0,1) = (a12 + a21) / S;
        quat(0,2) = (a13 + a31) /S;
    } else if (a22 > a33){
        float S = pow(1 + a22 - a11 - a33,0.5) * 2;
        quat(0,3) = (a13 - a31) / S;
        quat(0,0) = (a12 + a21) / S;
        quat(0,1) = 0.25*S;
        quat(0,2) = (a23 + a32) / S;
    } else {
        float S =pow(1.0 + a33 - a11 - a22,0.5) * 2;
        quat(0,3) = (a21 - a12) /S;
        quat(0,0) = (a13 + a31)/S;
        quat(0,1) = (a23 + a32)/S;
        quat(0,2) = 0.25*S;
    }

    return quat;


}

MatrixXd youbot_kinematic::convert_quat2rodrigues(MatrixXd quat)
{

    double ux;
    double uy;
    double uz;
    MatrixXd rod(1,3);

    double theta = 2*acos(quat(0,3));
    if (theta != 0){
        ux = quat(0,0) / sin(theta/2);
        uy = quat(0,1) / sin(theta/2);
        uz = quat(0,2) / sin(theta/2);
    }
    else{
        ux = 0;
        uy = 0;
        uz = 0;
    }
    double X = ux * theta;
    double Y = uy * theta;
    double Z = uz * theta;
    
    rod(0,0) = X;
    rod(0,1) = Y;
    rod(0,2) = Z;
    
    return rod;
 
}


double* youbot_kinematic::inverse_kine_ite(Matrix4d pose, double joint_val[])
{
    //TODO: Fill in this function to complete the question 3d

    double* curr_joint = joint_val; // = malloc(sizeof(joint_val));   //curr_joint[5];
    Eigen::Matrix4d curr_T;
    MatrixXd curr_J(6,5);
    MatrixXd rotmat(3,3);
    MatrixXd quat(1,4);
    MatrixXd rod(1,3);
    MatrixXd Xe(6,1);
    MatrixXd curr_Xe(6,1);
    MatrixXd pseudoInv;
    MatrixXd curr_J_T;
    MatrixXd A_T;
    MatrixXd result;
    MatrixXd temp(5,1);
    double error=2;
  
  
    //std::cout << "pose\n"<< pose << "\n" << std::endl;
    rotmat(0,0) = pose(0,0);
    rotmat(0,1) = pose(0,1);
    rotmat(0,2) = pose(0,2);
    rotmat(1,0) = pose(1,0);
    rotmat(1,1) = pose(1,1);
    rotmat(1,2) = pose(1,2);
    rotmat(2,0) = pose(2,0);
    rotmat(2,1) = pose(2,1);
    rotmat(2,2) = pose(2,2);
    //std::cout << "rotmat\n"<< rotmat << "\n" << std::endl;

    quat = youbot_kinematic::convert_rotmat2quat(rotmat);
    rod = youbot_kinematic::convert_quat2rodrigues(quat);
    //std::cout << "rod\n"<< rod << "\n" << std::endl;

    Xe(0,0) = pose(0,3);
    Xe(1,0) = pose(1,3);
    Xe(2,0) = pose(2,3);
    Xe(3,0) = rod(0,0);
    Xe(4,0) = rod(0,1);
    Xe(5,0) = rod(0,2);
    //std::cout << "Xe\n"<< Xe << "\n" << std::endl;
 
    int i = 0;
   
    joint_val[0]= joint_val[1] = joint_val[2] = joint_val[3] = joint_val[4] =0.0;
    
    while( error>0.0001){
        curr_T = youbot_kinematic::forward_kine(joint_val, 5);
        //std::cout << "curr_T\n"<< curr_T << "\n" << std::endl;
        curr_J = youbot_kinematic::get_jacobian(joint_val);
        //std::cout << "curr_J\n"<< curr_J << "\n" << std::endl;
        rotmat(0,0) = curr_T(0,0);
        rotmat(0,1) = curr_T(0,1);
        rotmat(0,2) = curr_T(0,2);
        rotmat(1,0) = curr_T(1,0);
        rotmat(1,1) = curr_T(1,1);
        rotmat(1,2) = curr_T(1,2);
        rotmat(2,0) = curr_T(2,0);
        rotmat(2,1) = curr_T(2,1);
        rotmat(2,2) = curr_T(2,2);
        //std::cout << "rotmat\n"<< rotmat << "\n" << std::endl;
        curr_Xe(0,0) = curr_T(0,3);
	curr_Xe(1,0) = curr_T(1,3);
	curr_Xe(2,0) = curr_T(2,3);
        quat = youbot_kinematic::convert_rotmat2quat(rotmat);
        rod = youbot_kinematic::convert_quat2rodrigues(quat);
        //std::cout << "rod\n"<< rod << "\n" << std::endl;
	curr_Xe(3,0) = rod(0,0);
	curr_Xe(4,0) = rod(0,1);
	curr_Xe(5,0) = rod(0,2);
        //std::cout << "curr_Xe\n"<< curr_Xe << "\n" << std::endl;
        
        //
        curr_J_T = curr_J.transpose();
        //std::cout << "curr_J_T\n"<< curr_J_T << "\n" << std::endl;

        //pseudoInv = curr_J_T * (curr_J * curr_J_T ).inverse();
        pseudoInv = ( (curr_J_T * curr_J) ).inverse() * curr_J_T;
        //std::cout << "pseudoInv\n"<< pseudoInv << "\n" << std::endl;
        temp(0,0) = joint_val[0];
        //std::cout << "temp(0)\n"<< temp(0) << "\n" << std::endl;
        temp(1,0) = joint_val[1];
        temp(2,0) = joint_val[2];
        temp(3,0) = joint_val[3];
        temp(4,0) = joint_val[4];
        //std::cout << "temp\n"<< temp << "\n" << std::endl;
 
        result = temp + 0.001*pseudoInv*(Xe-curr_Xe);
        error = ((Xe-curr_Xe)).norm();
        
        //std::cout << "error\n"<< error << "\n" << std::endl;
        

        joint_val[0] = result(0,0);
        joint_val[1] = result(1,0);
        joint_val[2] = result(2,0);
        joint_val[3] = result(3,0);
        joint_val[4] = result(4,0);
        //std::cout << "joint_val\n"<< joint_val << "\n" << std::endl;
        i++; 
    }
    //std::cout << "curr_Xe\n"<< curr_Xe << "\n" << std::endl;
    //std::cout << "iterative inverse kinematic results for the pose below\n"<< result << "\n" << std::endl;
    //std::cout << "error\n"<< error << "\n" << std::endl;

    
    
    return curr_joint;
    


}


bool youbot_kinematic::check_singularity(double joint_val[])
{
    //TODO: Fill in this function to complete the question 3e
    bool checker = false;
    double determinant;
    MatrixXd ja(6,5);
   
    ja = youbot_kinematic::get_jacobian(joint_val);
    determinant = ((ja.transpose() * ja  )).determinant();
    if (determinant == 0){
        // the sigularity exists
        checker = true;

    }
    
    
    return  checker;
    
}
