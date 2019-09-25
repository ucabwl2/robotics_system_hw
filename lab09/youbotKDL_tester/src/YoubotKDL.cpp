#include <youbotKDL_tester/YoubotKDL.h>

void YoubotKDL::init()
{
    DH_params[0][0] = -0.033;   DH_params[1][0] = 0.155;  DH_params[2][0] = 0.135; DH_params[3][0] = -0.002;  DH_params[4][0] = 0.0;
    DH_params[0][1] = M_PI_2;   DH_params[1][1] = 0.0;    DH_params[2][1] = 0.0;   DH_params[3][1] = M_PI_2;  DH_params[4][1] = M_PI;
    DH_params[0][2] = 0.145;    DH_params[1][2] = 0.0;    DH_params[2][2] = 0.0;   DH_params[3][2] = 0.0;     DH_params[4][2] = -0.185;
    DH_params[0][3] = M_PI;     DH_params[1][3] = M_PI_2; DH_params[2][3] = 0.0;   DH_params[3][3] = -M_PI_2; DH_params[4][3] = -M_PI;

    DH_params[0][3] = 170*M_PI/180;
    DH_params[1][3] = 65*M_PI/180+M_PI_2;
    DH_params[2][3] = -146*M_PI/180;
    DH_params[3][3] = M_PI_2+102.5*M_PI/180;
    DH_params[4][3] = M_PI+167.5*M_PI/180;
    DH_params[0][4] = DH_params[1][4] = DH_params[2][4] = DH_params[3][4] = DH_params[4][4] = -1;

    joint_offset[0] = 170*M_PI/180;
    joint_offset[1] = 65*M_PI/180;
    joint_offset[2] = -146*M_PI/180;
    joint_offset[3] = 102.5*M_PI/180;
    joint_offset[4] = 167.5*M_PI/180;

    joint_limit_min[0] = -169*M_PI/180;
    joint_limit_min[1] = -65*M_PI/180;
    joint_limit_min[2] = -151*M_PI/180;
    joint_limit_min[3] = -102.5*M_PI/180;
    joint_limit_min[4] = -167.5*M_PI/180;

    joint_limit_max[0] = 169*M_PI/180;
    joint_limit_max[1] = 90*M_PI/180;
    joint_limit_max[2] = 146*M_PI/180;
    joint_limit_max[3] = 102.5*M_PI/180;
    joint_limit_max[4] = 167.5*M_PI/180;

    this->subscriber_joint_state = this->n.subscribe<sensor_msgs::JointState>("/joint_states", 5, &YoubotKDL::joint_state_callback,
                                                                              this);
    setup_kdl_chain();
    this->current_joint_position = KDL::JntArray(this->kine_chain.getNrOfJoints());
}

KDL::Jacobian YoubotKDL::get_jacobian(KDL::JntArray current_joint_position)
{
    KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(this->kine_chain);
    KDL::Jacobian jac = KDL::Jacobian(this->kine_chain.getNrOfJoints());
    jac_solver.JntToJac(current_joint_position, jac);
    return jac;
}

KDL::Frame YoubotKDL::forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose)
{
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(this->kine_chain);
    fk_solver.JntToCart(current_joint_position, current_pose, 5);
    return current_pose;
}

void YoubotKDL::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for (int i = 0; i < 5; i++)
        this->current_joint_position.data(i) = q->position.at(i);
}

void YoubotKDL::setup_kdl_chain() {
    for (int i = 0; i < 5; i++)
        this->kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Vector(), KDL::Vector(0, 0, DH_params[i][4]), KDL::Joint::RotAxis),
                                                 KDL::Frame::DH(DH_params[i][0], DH_params[i][1], DH_params[i][2],
                                                                DH_params[i][3])));
}

KDL::JntArray YoubotKDL::inverse_kinematics_closed(KDL::Frame desired_pose)
{

    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(this->kine_chain);
    KDL::JntArray required_joint = KDL::JntArray(this->kine_chain.getNrOfJoints());
    ik_solver.CartToJnt(this->current_joint_position, desired_pose, required_joint);

    return required_joint;
}

void YoubotKDL::broadcast_pose(KDL::Frame current_pose)
{
    geometry_msgs::TransformStamped trans;

    trans = tf2::kdlToTransform(current_pose);
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "base_link";
    trans.child_frame_id = "arm_end_effector";

    this->pose_broadcaster.sendTransform(trans);

}