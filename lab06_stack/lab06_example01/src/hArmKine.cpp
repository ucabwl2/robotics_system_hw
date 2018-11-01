#include "lab06_example01/hArmKine.h"

void hArm_kinematic::init()
{
    DH_params[0][0] = 0.0;     DH_params[1][0] = 0.26569;                    DH_params[2][0] = 0.03;              DH_params[3][0] = 0.0;     DH_params[4][0] = 0.0;     DH_params[5][0] = 0.0;
    DH_params[0][1] = -M_PI_2; DH_params[1][1] = 0.0;                        DH_params[2][1] = -M_PI_2;           DH_params[3][1] = -M_PI_2; DH_params[4][1] = -M_PI_2; DH_params[5][1] = 0.0;
    DH_params[0][2] = 0.159;   DH_params[1][2] = 0.0;                        DH_params[2][2] = 0.0;               DH_params[3][2] = 0.258;   DH_params[4][2] = 0.0;     DH_params[5][2] = -0.123;
    DH_params[0][3] = 0.0;     DH_params[1][3] = -M_PI_2 + atan(0.03/0.264); DH_params[2][3] = -atan(0.03/0.264); DH_params[3][3] = 0.0;     DH_params[4][3] = 0.0;     DH_params[5][3] = 0.0;

    joint_limit_min[0] = -M_PI;
    joint_limit_min[1] = -M_PI_2;
    joint_limit_min[2] = -M_PI_2;
    joint_limit_min[3] = -M_PI;
    joint_limit_min[4] = -M_PI_2;
    joint_limit_min[5] = -M_PI;

    joint_limit_max[0] = M_PI;
    joint_limit_max[1] = M_PI_2;
    joint_limit_max[2] = 3*M_PI_4;
    joint_limit_max[3] = M_PI;
    joint_limit_max[4] = M_PI_2;
    joint_limit_max[5] = M_PI;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &hArm_kinematic::joint_state_callback, this);
    traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);
}

void hArm_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for(int i = 0; i < 6; i++)
        current_joint_position[i] = q->position.at(i);

    current_pose = forward_kine(current_joint_position, 6);
}

Matrix4d hArm_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
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

Matrix4d hArm_kinematic::forward_kine(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        if ((i == 5) || (i == 4))
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], -joint_val[i] + DH_params[i][3]);
        else
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_val[i] + DH_params[i][3]);

        T = T * A;
    }

    return T;
}

void hArm_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "base_link";
    T.child_frame_id = "arm_end_effector";

    pose_br.sendTransform(T);
}

void hArm_kinematic::publish_joint_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory, int tfs)
{

    //This function can be used to publish joint trajectory. Or you can implement it yourself.
    trajectory_msgs::JointTrajectory msg;

    msg.header.stamp = ros::Time::now();
    msg.joint_names.push_back("joint1");
    msg.joint_names.push_back("joint2");
    msg.joint_names.push_back("joint3");
    msg.joint_names.push_back("joint4");
    msg.joint_names.push_back("joint5");
    msg.joint_names.push_back("joint6");

    joint_trajectory.time_from_start.nsec = tfs;

    msg.points.push_back(joint_trajectory);

    this->traj_publisher.publish(msg);
}


MatrixXd hArm_kinematic::get_jacobian(double joint_val[])
{
    //SINCE THE IMPLEMENTATION OF THIS FUNCTION IS VERY SIMILAR TO THE EXERCISES IN THE LAB, THIS IS LEFT BLANK ON PURPOSE.
}

double* hArm_kinematic::inverse_kine_ite(Matrix4d pose, double joint_val[])
{
    //SINCE THE IMPLEMENTATION OF THIS FUNCTION IS VERY SIMILAR TO THE EXERCISES IN THE LAB, THIS IS LEFT BLANK ON PURPOSE.
}

bool hArm_kinematic::check_singularity(double joint_val[])
{
    //SINCE THE IMPLEMENTATION OF THIS FUNCTION IS VERY SIMILAR TO THE EXERCISES IN THE LAB, THIS IS LEFT BLANK ON PURPOSE.
}

MatrixXd hArm_kinematic::inverse_kine_closed_form(Matrix4d pose)
{

    //There are 8 possible inverse kinematic solutions. The last column indicates whether or not the solution lies within the robot workspace (does not exceed joint limit)
    MatrixXd inv_kine_sol(8, 7);

    double a2 = DH_params[1][0];
    double a3 = DH_params[2][0];
    double d1 = DH_params[0][2];
    double d6 = DH_params[5][2];
    double d4 = DH_params[3][2];
    double beta = atan2(d4, a3);

    //Solve for theta1
    inv_kine_sol(0, 0) = inv_kine_sol(1, 0) = inv_kine_sol(2, 0) = inv_kine_sol(3, 0) = atan2(pose(1, 3), pose(0, 3));

    //Check with the joint limit for the other solution.
    if (inv_kine_sol(0, 0) <= 0.0)
        inv_kine_sol(4, 0) = inv_kine_sol(5, 0) = inv_kine_sol(6, 0) = inv_kine_sol(7, 0) = atan2(pose(1, 3), pose(0, 3)) + M_PI;
    else
        inv_kine_sol(4, 0) = inv_kine_sol(5, 0) = inv_kine_sol(6, 0) = inv_kine_sol(7, 0) = atan2(pose(1, 3), pose(0, 3)) - M_PI;

    double K, A1, A2, B1, B2;

    //Solve for theta3
    for (int i = 0; i < 2; i++)
    {
        if (sin(inv_kine_sol(4*i, 0)) == 0)
            K = pow((pose(0, 3) - d6*pose(0, 2))/(cos(inv_kine_sol(4*i, 0))), 2) + pow(pose(2, 3) - d1 - d6*pose(2, 2), 2) - pow(a3, 2) - pow(a2, 2) - pow(d4, 2);
        else
            K = pow((pose(1, 3) - d6*pose(1, 2))/(sin(inv_kine_sol(4*i, 0))), 2) + pow(pose(2, 3) - d1 - d6*pose(2, 2), 2) - pow(a3, 2) - pow(a2, 2) - pow(d4, 2);

        K = K/(2*a2*sqrt(pow(a3, 2) + pow(d4, 2)));

        inv_kine_sol(4*i, 2) = inv_kine_sol(4*i + 1, 2) = acos(K)-beta;
        inv_kine_sol(4*i + 2, 2) = inv_kine_sol(4*i + 3, 2) = -acos(K) - beta;

        B1 = a2 + a3*cos(inv_kine_sol(4*i, 2)) - d4*sin(inv_kine_sol(4*i, 2));
        B2 = a2 + a3*cos(inv_kine_sol(4*i + 2, 2)) - d4*sin(inv_kine_sol(4*i + 2, 2));

        A1 = a3*sin(inv_kine_sol(4*i, 2)) + d4*cos(inv_kine_sol(4*i, 2));
        A2 = a3*sin(inv_kine_sol(4*i + 2, 2)) + d4*cos(inv_kine_sol(4*i + 2, 2));

        //Solve for theta2
        if (sin(inv_kine_sol(4*i, 0)) == 0)
        {
            inv_kine_sol(4*i, 1) = inv_kine_sol(4*i + 1, 1) = atan2(pose(2, 3) - d1 - d6*pose(2, 2), (pose(0, 3) - d6*pose(0, 2))/(cos(inv_kine_sol(4*i, 0)))) - atan2(B1, A1);
            inv_kine_sol(4*i + 2, 1) = inv_kine_sol(4*i + 3, 1) = atan2(pose(2, 3) - d1 - d6*pose(2, 2), (pose(0, 3) - d6*pose(0, 2))/(cos(inv_kine_sol(4*i, 0)))) - atan2(B2, A2);
        }
        else
        {
            inv_kine_sol(4*i, 1) = inv_kine_sol(4*i + 1, 1) = atan2(pose(2, 3) - d1 - d6*pose(2, 2), (pose(1, 3) - d6*pose(1, 2))/(sin(inv_kine_sol(4*i, 0)))) - atan2(B1, A1);
            inv_kine_sol(4*i + 2, 1) = inv_kine_sol(4*i + 3, 1) = atan2(pose(2, 3) - d1 - d6*pose(2, 2), (pose(1, 3) - d6*pose(1, 2))/(sin(inv_kine_sol(4*i, 0)))) - atan2(B2, A2);

        }

    }

    for (int i = 0; i < 7; i++)
    {
        Matrix4d pose36 = (dh_matrix_standard(DH_params[0][0], DH_params[0][1], DH_params[0][2], inv_kine_sol(i, 0))*
                           dh_matrix_standard(DH_params[1][0], DH_params[1][1], DH_params[1][2], inv_kine_sol(i, 1))*
                           dh_matrix_standard(DH_params[2][0], DH_params[2][1], DH_params[2][2], inv_kine_sol(i, 2))).inverse()*pose;

        //Solve for theta5
        if (i%2 == 0)
            inv_kine_sol(i, 4) = acos(-pose36(2, 2));
        else
            inv_kine_sol(i, 4) = -acos(-pose36(2, 2));

        //Solve for theta4
        inv_kine_sol(i, 3) = atan2(pose36(1, 2)/(-sin(inv_kine_sol(i, 4))), pose36(0, 2)/(-sin(inv_kine_sol(i, 4))));
        //Solve for theta6
        inv_kine_sol(i, 5) = atan2(pose36(2, 1)/sin(inv_kine_sol(i, 4)), pose36(2, 0)/(-sin(inv_kine_sol(i, 4))));
    }

    //Check if the inverse kinematic solutions are in the robot workspace
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if ((inv_kine_sol(i, j) + DH_params[j][3] <= joint_limit_min[j]) || (inv_kine_sol(i, j) + DH_params[j][3] >= joint_limit_max[j]))
            {
                //Put some value there to let you know that this solution is not valid.
                inv_kine_sol(i, 6) = -j;
                break;
            }
            else
            {
                //Put some value there to let you know that this solution is valid.
                inv_kine_sol(i, 6) = 1;
            }
        }
    }

    return inv_kine_sol;
}
