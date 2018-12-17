#include <kdl_kine/kdl_kine_solver.h>
#include <string>

void iiwa14_kinematic::init()
{
  
    subscriber_joint_state = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &iiwa14_kinematic::joint_state_callback, this);

    current_joint_position.resize(7);


    setup_kdl_chain();
    std::string base_link = "iiwa_link_0";
    std::string ee_link = "iiwa_link_ee";

    kdl_tree.getChain(base_link, ee_link, kine_chain);

    KDL_joint = KDL::JntArray(kine_chain.getNrOfJoints());
}

void iiwa14_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for (int i = 0; i < 7; i++)
        this->current_joint_position.data(i) = q->position.at(i);
}


void iiwa14_kinematic::setup_kdl_chain() {
    if (!kdl_parser::treeFromParam("robot_description", kdl_tree))
        ROS_ERROR("Failed to construct kdl tree");
}

KDL::Frame iiwa14_kinematic::KDLfkine(KDL::JntArray current_joint_position)
{
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(this->kine_chain);
    KDL::Frame pose;
    fk_solver.JntToCart(current_joint_position, pose, 7);
    return pose;
}

KDL::Jacobian iiwa14_kinematic::KDLjacob(KDL::JntArray current_joint_position)
{
    KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(this->kine_chain);
    KDL::Jacobian jac = KDL::Jacobian(this->kine_chain.getNrOfJoints());
    jac_solver.JntToJac(current_joint_position, jac);
    return jac;
}

KDL::JntArray iiwa14_kinematic::inverse_kinematics_closed(KDL::Frame desired_pose)
{

    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(this->kine_chain);
    KDL::JntArray required_joint = KDL::JntArray(this->kine_chain.getNrOfJoints());
    ik_solver.CartToJnt(this->current_joint_position, desired_pose, required_joint);
    return required_joint;
}

MatrixXd iiwa14_kinematic::getB(VectorXd joint_val)
{
    KDL::JntArray q;
    KDL::JntSpaceInertiaMatrix KDL_B = KDL::JntSpaceInertiaMatrix(7);
    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    q.resize(7);
    for (int i = 0; i < 7; i++)
        q.data(i) = joint_val(i);

    dyn.JntToMass(q, KDL_B);

    return KDL_B.data;
}


VectorXd iiwa14_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{

    KDL::JntArray q, q_vel;
    KDL::JntArray KDL_C = KDL::JntArray(7);


    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    q.resize(7);
    q_vel.resize(7);
    for (int i = 0; i < 7; i++){
        q.data(i) = joint_val(i);
        q_vel.data(i) = joint_vel(i);
    }


    dyn.JntToCoriolis(q, q_vel, KDL_C);

    return KDL_C.data;
}


VectorXd iiwa14_kinematic::getG(VectorXd joint_val)
{
    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    KDL::JntArray q;
    KDL::JntArray gravity = KDL::JntArray(7);

    q.resize(7);
    for (int i = 0; i < 7; i++)
        q.data(i) = joint_val(i);

    dyn.JntToGravity(q, gravity);

    return gravity.data;
}
