#include <cw3q2/iiwa14Kine.h>


void iiwa14_kinematic::init()
{
    //Delete this and fill DH parameters based on the xacro file (cw3/iiwa_description/urdf/iiwa14.xacro).
    //for (int i = 0; i < 7;i++)
    //    for (int j = 0; j < 4;j++)
    //        DH_params[i][j] = 0.0;

    //a
    DH_params[0][0] = 0.0;    DH_params[1][0] = 0.0;    DH_params[2][0] = 0.0;   DH_params[3][0] = 0.0;     DH_params[4][0] = 0.0; DH_params[5][0] = 0.0; DH_params[6][0] = 0.0;
    //alpha
    DH_params[0][1] = -M_PI_2;   DH_params[1][1] = M_PI_2;    DH_params[2][1] = M_PI_2;   DH_params[3][1] = -M_PI_2;  DH_params[4][1] = -M_PI_2; DH_params[5][1] = M_PI_2; DH_params[6][1] = 0;
    //d
    DH_params[0][2] = 0.2025;   DH_params[1][2] = 0;  DH_params[2][2] = 0.42; DH_params[3][2] = 0;  DH_params[4][2] = 0.4; DH_params[5][2] = 0.0; DH_params[6][2] = 0.126;
    //theta
    DH_params[0][3] = 0.0;     DH_params[1][3] = 0.0; DH_params[2][3] = 0.0;   DH_params[3][3] = 0.0; DH_params[4][3] = 0.0; DH_params[5][3] = 0.0; DH_params[6][3] = 0.0;

    joint_limit_min[0] = -170*M_PI/180;  //-2.967
    joint_limit_min[1] = -120*M_PI/180;  //-2.0943
    joint_limit_min[2] = -170*M_PI/180;  //-2.967
    joint_limit_min[3] = -120*M_PI/180;  //-2.0943
    joint_limit_min[4] = -170*M_PI/180;  //-2.967
    joint_limit_min[5] = -120*M_PI/180;  //-2.0943
    joint_limit_min[6] = -175*M_PI/180;  //-3.0543

    joint_limit_max[0] = 170*M_PI/180;  //2.967
    joint_limit_max[1] = 120*M_PI/180;  //2.0943
    joint_limit_max[2] = 170*M_PI/180;  //2.967
    joint_limit_max[3] = 120*M_PI/180;  //2.0943
    joint_limit_max[4] = 170*M_PI/180;  //2.967
    joint_limit_max[5] = 120*M_PI/180;  //2.0943
    joint_limit_max[6] = 175*M_PI/180;  //3.0543

    //The mass of each link.
    mass.resize(7);
    mass << 4, 4, 3, 2.7, 1.7, 1.8, 0.3;

    //Moment on inertia of each link.
    //Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
    Ixyz.resize(7, 3);
    Ixyz << 0.1, 0.09, 0.02,
            0.05, 0.018, 0.044,
            0.08, 0.075, 0.01,
            0.03, 0.01, 0.029,
            0.02, 0.018, 0.005,
            0.005, 0.0036, 0.0047,
            0.001, 0.001, 0.001;

    

    //gravity
    g = 9.8;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &iiwa14_kinematic::joint_state_callback, this);
}


void iiwa14_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    VectorXd J(7);

    for(int i = 0; i < 7; i++)
        J(i) = q->position.at(i);

    current_pose = forward_kine(J, 7);
    broadcast_pose(current_pose);
}

Matrix4d iiwa14_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
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

void iiwa14_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "iiwa_link_0";
    T.child_frame_id = "iiwa_ee";

    pose_br.sendTransform(T);
}

//Useful Transformation function.
Matrix4d T_translation(Vector3d t)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    for (int i = 0; i < 3; i++)
        T(i, 3) = t(i);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationZ(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationY(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 2) = sin(theta);
    T(2, 0) = -sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationX(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(1, 1) = cos(theta);
    T(1, 2) = -sin(theta);
    T(2, 1) = sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

Matrix4d iiwa14_kinematic::forward_kine(VectorXd joint_val, int frame)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    
    //Add offset from the iiwa platform.
    T(2, 3) = 0.1575;
    //TODO: Fill in this function to complete Q2.
    Matrix4d A;
    for(int i = 0;i < frame; i++)
    {
        A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_val[i] + DH_params[i][3]);

        T = T * A;
    }

    return T;
}

MatrixXd iiwa14_kinematic::forward_kine_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete Q2.
    Matrix4d Timinus1 = Matrix4d::Identity(4, 4);
    Matrix4d Ti = Matrix4d::Identity(4, 4);
    Matrix4d T_cm = Matrix4d::Identity(4, 4);
    Matrix4d Rotation_Z = Matrix4d::Identity(4, 4);
    Vector3d t(3,1);
    Matrix4d T_tran = Matrix4d::Identity(4, 4); 

    MatrixXd linkcm(7,3);
    linkcm(0,0) = 0;       linkcm(0,1) = -0.03;   linkcm(0,2) = 0.2775-0.1575; 
    linkcm(1,0) = -0.0003; linkcm(1,1) = -0.059;  linkcm(1,2) = 0.042; 
    linkcm(2,0) = 0.0;     linkcm(2,1) = 0.03;    linkcm(2,2) = 0.3345; 
    linkcm(3,0) = 0.0;     linkcm(3,1) = 0.067;   linkcm(3,2) = 0.034;
    linkcm(4,0) = -0.0001; linkcm(4,1) = -0.021;  linkcm(4,2) = 0.2605;
    linkcm(5,0) = 0.0;     linkcm(5,1) = -0.0006; linkcm(5,2) = 0.0004; 
    linkcm(6,0) = 0.0;     linkcm(6,1) = 0.0;     linkcm(6,2) = 0.101;  

    //Add offset from the iiwa platform.

    if((frame-1)==0)
    {
        Timinus1 = Matrix4d::Identity(4, 4);
    }
    else
    {
        Timinus1 = forward_kine(joint_val, frame-1);
    }
        

    // Timinus1 = forward_kine(joint_val, frame-1);

    // std::cout << "Timinus1 \n"<< Timinus1 << "\n" << std::endl;

    // Ti = forward_kine(joint_val, frame);
    // std::cout << "Ti \n"<< Ti << "\n" << std::endl;
    
    Rotation_Z = T_rotationZ(joint_val[frame-1] + DH_params[frame-1][3]);
    
    // std::cout << "Rotation_Z \n"<< Rotation_Z << "\n" << std::endl;


    for (int i = 0; i < 3; i++)
        t(i) = linkcm(frame-1, i);
    // std::cout << "t \n"<< t << "\n" << std::endl;
    T_tran = T_translation(t);

    T_cm = Timinus1 * Rotation_Z * T_tran;
    // std::cout << "T_tran \n"<< T_tran << "\n" << std::endl;
    // std::cout << "T_cm \n"<< T_cm << "\n" << std::endl;

    

    return T_cm ;
}


VectorXd get_frame_joint_val(VectorXd joint_val, int frame)
{
    int th=frame;
    VectorXd v(frame);
    for (int i = 0; i < frame; i++)
        v(i) = joint_val(i);
    // std::cout << "v \n"<< v << "\n" << std::endl;
    return v;
}


MatrixXd iiwa14_kinematic::get_jacobian(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
    // Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d Timinus1 = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d Te = Eigen::Matrix4d::Identity(4, 4);
    Vector3d z0;
    Vector3d zi;
    Vector3d p0;
    Vector3d pi;
    Vector3d pe;
    Vector3d temp;
    z0(0) = 0;
    z0(1) = 0;
    z0(2) = 1;
    p0(0) = 0;
    p0(1) = 0;
    p0(2) = 0.1575;
    MatrixXd ja = Eigen::MatrixXd::Constant(6, 7, 0);
    Te = iiwa14_kinematic::forward_kine(joint_val, 7);
    for (int j = 0; j < 3; j++)
        {
            pe(j) = Te(j,3);
        }
    

    // VectorXd v(frame);
    VectorXd v(7);
    for (int i = 0; i < 7; i++)
    {
        v.resize(i+1);
        v = get_frame_joint_val(joint_val, i+1);
        // Ti = iiwa14_kinematic::forward_kine(v, i+1);
        if(i==0)
        {
            zi = z0;
            pi = p0;
        }
        else
        {
            Timinus1 = iiwa14_kinematic::forward_kine(v, i);
            for (int j = 0; j < 3; j++)
            {
                zi(j) = Timinus1(j,2);
                pi(j) = Timinus1(j,3);
            }
        }
        for (int k = 3; k < 6; k++)
            ja(k,i) =  zi(k-3);
        for (int h = 0; h < 3; h++)
        {
            temp = zi.cross(pe-pi);
            ja(h,i) =  temp(h); //c(h);
        }
        // std::cout << "v \n"<< v << "\n" << std::endl;
    }


    return ja;


}


MatrixXd iiwa14_kinematic::get_jacobian_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete Q2.
    Eigen::Matrix4d T_cm = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T_cm_1 = Eigen::Matrix4d::Identity(4, 4);
    VectorXd v(frame);
    Vector3d z0;
    Vector3d p0;
    Vector3d pi;
    Vector3d temp;
    Vector3d zi;
    Vector3d pli;
    Vector3d pj_1;
    z0(0) = 0;
    z0(1) = 0;
    z0(2) = 1;
    // std::cout << "z0 \n"<< z0 << "\n" << std::endl;
    p0(0) = 0;
    p0(1) = 0;
    p0(2) = 0.1575;
    // std::cout << "joint_val \n"<< joint_val << "\n" << std::endl;
    v = get_frame_joint_val(joint_val,frame);
    T_cm = iiwa14_kinematic::forward_kine_cm(v, frame);
    for (int i = 0; i < 3; i++)
        pli(i) =  T_cm(i,3);
    
    // std::cout << "T_cm \n"<< T_cm << "\n" << std::endl;
    // std::cout << "pli \n"<< pli << "\n" << std::endl;

    MatrixXd ja_cm = Eigen::MatrixXd::Constant(6, 7, 0);
    // std::cout << "ja_cm \n"<< ja_cm << "\n" << std::endl;

    // std::cout << "z0.cross(pli) \n"<< z0.cross(pli) << "\n" << std::endl; 

    for (int i = 0; i<frame; i++)
    {
        // std::cout << "frame "<< (i+1) << "\n" << std::endl;
        // std::cout << "pli "<< pli << "\n" << std::endl;
        v = get_frame_joint_val(joint_val,i+1);
        // std::cout << "v \n"<< v << "\n" << std::endl;
        // T_cm = iiwa14_kinematic::forward_kine_cm(v, i+1);
        if(i==0)
        {
            pi = p0;
            zi = z0;
        }
        else
        {
            T_cm_1 = iiwa14_kinematic::forward_kine(v, i);
            for (int j = 0; j < 3; j++)
            {
                pi(j) =  T_cm_1(j,3);
                zi(j) = T_cm_1(j,2);
            }
                
        }
        for (int k = 3; k < 6; k++)
            ja_cm(k,i) =  zi(k-3);
        for (int h = 0; h < 3; h++)
        {
            temp = zi.cross(pli-pi);
            ja_cm(h,i) =  temp(h); //c(h);
        }
        // std::cout << "T_cm \n"<< T_cm << "\n" << std::endl;
        // std::cout << "T_cm_1 \n"<< T_cm_1 << "\n" << std::endl;
        // std::cout << "pi \n"<< pi << "\n" << std::endl;
        // std::cout << "z0 \n"<< z0 << "\n" << std::endl;

    }

    return ja_cm;
}


MatrixXd iiwa14_kinematic::convert_rotmat2quat(Matrix3d rotmat)
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


MatrixXd iiwa14_kinematic::convert_quat2rodrigues(MatrixXd quat)
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



VectorXd iiwa14_kinematic::inverse_kine_ite(Matrix4d pose, VectorXd joint_val)
{

    //TODO: Fill in this function to complete Q2.
    VectorXd curr_joint = joint_val; // = malloc(sizeof(joint_val));   //curr_joint[5];
    Eigen::Matrix4d curr_T;
    MatrixXd curr_J(6,7);
    MatrixXd rotmat(3,3);
    MatrixXd quat(1,4);
    MatrixXd rod(1,3);
    MatrixXd Xe(6,1);
    MatrixXd curr_Xe(6,1);
    MatrixXd pseudoInv;
    MatrixXd curr_J_T;
    MatrixXd A_T;
    MatrixXd result;
    MatrixXd temp(7,1);
    double error=2;
  
  
    // std::cout << "pose\n"<< pose << "\n" << std::endl;
    rotmat(0,0) = pose(0,0);
    rotmat(0,1) = pose(0,1);
    rotmat(0,2) = pose(0,2);
    rotmat(1,0) = pose(1,0);
    rotmat(1,1) = pose(1,1);
    rotmat(1,2) = pose(1,2);
    rotmat(2,0) = pose(2,0);
    rotmat(2,1) = pose(2,1);
    rotmat(2,2) = pose(2,2);
    // std::cout << "rotmat\n"<< rotmat << "\n" << std::endl;

    quat = iiwa14_kinematic::convert_rotmat2quat(rotmat);
    rod = iiwa14_kinematic::convert_quat2rodrigues(quat);
    // std::cout << "rod\n"<< rod << "\n" << std::endl;

    Xe(0,0) = pose(0,3);
    Xe(1,0) = pose(1,3);
    Xe(2,0) = pose(2,3);
    Xe(3,0) = rod(0,0);
    Xe(4,0) = rod(0,1);
    Xe(5,0) = rod(0,2);
    // std::cout << "Xe\n"<< Xe << "\n" << std::endl;
 
    int i = 0;
   
    joint_val(0)= joint_val(1) = joint_val(2) = joint_val(3) = joint_val(4) = joint_val(5) = joint_val(6) = 0.1;
    
    while( error>0.001){
        curr_T = iiwa14_kinematic::forward_kine(joint_val, 7);
        // std::cout << "curr_T\n"<< curr_T << "\n" << std::endl;
        curr_J = iiwa14_kinematic::get_jacobian(joint_val);
        // std::cout << "curr_J\n"<< curr_J << "\n" << std::endl;
        rotmat(0,0) = curr_T(0,0);
        rotmat(0,1) = curr_T(0,1);
        rotmat(0,2) = curr_T(0,2);  
        rotmat(1,0) = curr_T(1,0);
        rotmat(1,1) = curr_T(1,1);
        rotmat(1,2) = curr_T(1,2);
        rotmat(2,0) = curr_T(2,0);
        rotmat(2,1) = curr_T(2,1);
        rotmat(2,2) = curr_T(2,2);
        // std::cout << "rotmat\n"<< rotmat << "\n" << std::endl;
        curr_Xe(0,0) = curr_T(0,3);
	curr_Xe(1,0) = curr_T(1,3);
	curr_Xe(2,0) = curr_T(2,3);
        quat = iiwa14_kinematic::convert_rotmat2quat(rotmat);
        rod = iiwa14_kinematic::convert_quat2rodrigues(quat);
        // std::cout << "rod\n"<< rod << "\n" << std::endl;
	curr_Xe(3,0) = rod(0,0);
	curr_Xe(4,0) = rod(0,1);
	curr_Xe(5,0) = rod(0,2);
        // std::cout << "curr_Xe\n"<< curr_Xe << "\n" << std::endl;
        
        //
        curr_J_T = curr_J.transpose();
        // std::cout << "curr_J_T\n"<< curr_J_T << "\n" << std::endl;

        //pseudoInv = curr_J_T * (curr_J * curr_J_T ).inverse();
        // pseudoInv = ((curr_J_T * curr_J)).inverse() * curr_J_T;
        pseudoInv = curr_J_T * ((curr_J * curr_J_T)).inverse();
        // std::cout << "pseudoInv\n"<< pseudoInv << "\n" << std::endl;
        temp(0,0) = joint_val(0);
        temp(1,0) = joint_val(1);
        temp(2,0) = joint_val(2);
        temp(3,0) = joint_val(3);
        temp(4,0) = joint_val(4);
        temp(5,0) = joint_val(5);
        temp(6,0) = joint_val(6);
        // std::cout << "temp\n"<< temp << "\n" << std::endl;
 
        result = temp + 0.01*pseudoInv*(Xe-curr_Xe);
        error = ((Xe-curr_Xe)).norm();
        
        // std::cout << "error\n"<< error << "\n" << std::endl;
        

        joint_val(0) = result(0,0);
        joint_val(1) = result(1,0);
        joint_val(2) = result(2,0);
        joint_val(3) = result(3,0);
        joint_val(4) = result(4,0);
        joint_val(5) = result(5,0);
        joint_val(6) = result(6,0);
        // std::cout << "joint_val\n"<< joint_val << "\n" << std::endl;
        i++; 
    }
    // std::cout << "joint_val\n"<< joint_val << "\n" << std::endl;
    // std::cout << "Xe\n"<< Xe << "\n" << std::endl;
    // std::cout << "curr_Xe\n"<< curr_Xe << "\n" << std::endl;
    return joint_val;

}

MatrixXd iiwa14_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //TODO: Fill in this function to complete Q2.
    MatrixXd IK_closed(8, 7);


    Vector3d p02;
    double dbs = DH_params[0][2];
    double dse = DH_params[2][2];
    double dew = DH_params[4][2];
    double dwf = DH_params[6][2];
    p02 << 0,0,dbs;
    Vector3d p24;
    p24 << 0,dse,0;
    Vector3d p46;
    p46 << 0,0,dew;
    Vector3d p67;
    p67 << 0,0,dwf;
    Vector3d p07;
    Vector3d p26, p26_norm;
    double theV4, theV1, phi, theV2, theV3=0, theta1, theta2, theta3, theta4, theta5, theta6, theta7;
    VectorXd joint_val_V4(4),joint_val_V3(3), GC6(2), GC4(2), GC2(2);
    GC6 << 1, -1; 
    GC4 << 1, -1; 
    GC2 << 1, -1;

    Matrix4d T_V4 = Eigen::Matrix4d::Identity(4, 4);
    Matrix4d T_V3 = Eigen::Matrix4d::Identity(4, 4);
    Vector3d Vvsew,Vsew, offset;

    Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    Matrix3d R = Eigen::Matrix3d::Identity(3, 3);
    double psi = -1.77898;
    Matrix3d p26cross, As, Bs, Cs, Aw, Bw, Cw;
    offset << 0,0,0.1575;


    p07 = pose.block(0,3,3,1) - offset;
    // p07(2) = p07(2) - 0.1575;
    p26 = p07 - p02 - (pose.block(0,0,3,3)*p67);
    int index=0;
    for(int j=0; j<2; j++)
    {
    	for(int i=0; i<2; i++)
    	{
    		for(int k=0; k<2; k++)
    		{
			
				theV4 = GC4(i)*acos( ((pow(p26.norm(),2)) - pow(dse,2) - pow(dew,2)  ) / (2*dse*dew)    );
				// // if(p26.corss()) //how to know 1R0?
				theV1 = atan2(p26(1), p26(0));
				phi = acos( (   pow(dse,2) + pow(p26.norm(),2) - pow(dew,2)) /  (2*dse*p26.norm())  );
				theV2 = atan2( pow(  pow(p26(0),2)  + pow(p26(1),2) ,0.5), p26(2) )  + GC4(i)*phi;
				// theV2 = atan2( pow(  pow(p26(0),2)  + pow(p26(1),2) ,0.5), p26(2) )  + GC4*phi;
				// std::cout << "theV2\n"<< theV2 << "\n" << std::endl;

				joint_val_V4 << theV1,theV2,theV3,theV4;
				T_V4 = forward_kine(joint_val_V4, 4);
				joint_val_V3 << theV1,theV2,theV3;
				T_V3 = forward_kine(joint_val_V3, 3);

				p26_norm = p26 / p26.norm();
				p26cross << 0, -p26_norm(2), p26_norm(1),
				            p26_norm(2), 0, -p26_norm(0),
				            -p26_norm(1), p26_norm(0), 0;

				As = p26cross * T_V3.block(0,0,3,3);
				Bs = - p26cross * p26cross * T_V3.block(0,0,3,3);
				Cs = p26_norm* p26_norm.transpose() * T_V3.block(0,0,3,3);
				theta1 = atan2( GC2(k)*( As(1,1)*sin(psi)+Bs(1,1)*cos(psi)+Cs(1,1)) , GC2(k)*( As(0,1)*sin(psi)+Bs(0,1)*cos(psi)+Cs(0,1)) );
				theta2 = GC2(k)*acos( As(2,1)*sin(psi)+Bs(2,1)*cos(psi)+ Cs(2,1)  );
				theta3 = atan2( GC2(k)*(-As(2,2)*sin(psi)-Bs(2,2)*cos(psi)-Cs(2,2) )  ,GC2(k)*(-As(2,0)*sin(psi)-Bs(2,0)*cos(psi)-Cs(2,0) ) );
				theta4 = theV4;
				Matrix4d T34 = Matrix4d::Identity(4, 4);

				//Add offset from the iiwa platform.
				T34(2, 3) = 0.1575;  
				//TODO: Fill in this function to complete Q2.
				Matrix4d A;
				//////////////?????????????//////
				A = dh_matrix_standard(DH_params[3][0], DH_params[3][1], DH_params[3][2], theta4 + DH_params[3][3]);

				T34 = T34 * A;

				Aw = T34.block(0,0,3,3).transpose() * As.transpose() *  pose.block(0,0,3,3);
				Bw = T34.block(0,0,3,3).transpose() * Bs.transpose() * pose.block(0,0,3,3);
				Cw = T34.block(0,0,3,3).transpose() * Cs.transpose() * pose.block(0,0,3,3);

				theta5 = atan2(GC6(j)*( Aw(1,2)*sin(psi)+Bw(1,2)*cos(psi)+Cw(1,2)), GC6(j)*(Aw(0,2)*sin(psi)+Bw(0,2)*cos(psi)+Cw(0,2)  )  );
				theta6 = GC6(j)*acos( Aw(2,2)*sin(psi)+Bw(2,2)*cos(psi)+Cw(2,2)  );
				theta7 = atan2( GC6(j)*(Aw(2,1)*sin(psi)+Bw(2,1)*cos(psi)+Cw(2,1)), GC6(j)*(-Aw(2,0)*sin(psi)-Bw(2,0)*cos(psi)-Cw(2,0)) );

				IK_closed.row(index) << theta1,theta2,theta3, theta4,theta5,theta6,theta7;
				index +=1;
    		}
    	
    	}
    	
    }
		

    return IK_closed;

    
}

MatrixXd iiwa14_kinematic::getB(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
    MatrixXd linkcm(7,3);
    Matrix4d T_cm = Eigen::Matrix4d::Identity(4, 4);
    Matrix3d Rg = Eigen::Matrix3d::Identity(3, 3);
    Matrix3d Jli = Eigen::Matrix3d::Identity(3, 3);
    MatrixXd B = Eigen::MatrixXd::Constant(7, 7, 0); //use Eigen;;Zeros
    MatrixXd jacobian_cm(6,7);
    MatrixXd Jp(3,7);
    MatrixXd Jo(3,7);

    linkcm(0,0) = 0;       linkcm(0,1) = -0.03;   linkcm(0,2) = 0.2775-0.1575; 
    linkcm(1,0) = -0.0003; linkcm(1,1) = -0.059;  linkcm(1,2) = 0.042; 
    linkcm(2,0) = 0.0;     linkcm(2,1) = 0.03;    linkcm(2,2) = 0.3345; 
    linkcm(3,0) = 0.0;     linkcm(3,1) = 0.067;   linkcm(3,2) = 0.034;
    linkcm(4,0) = -0.0001; linkcm(4,1) = -0.021;  linkcm(4,2) = 0.2605;
    linkcm(5,0) = 0.0;     linkcm(5,1) = -0.0006; linkcm(5,2) = 0.0004; 
    linkcm(6,0) = 0.0;     linkcm(6,1) = 0.0;     linkcm(6,2) = 0.101;  

    MatrixXd I_parallel(3,3);
    for (int i =0; i<7; i++)
    {
        I_parallel(0,0) = Ixyz(i,0) ;//+ mass(i)*(pow(linkcm(i,1),2) + pow(linkcm(i,2),2));
        I_parallel(0,1) = 0; //-mass(i)*linkcm(i,0)*linkcm(i,1);
        I_parallel(0,2) = 0; //- mass(i)*linkcm(i,0)*linkcm(i,2);
        I_parallel(1,0) = 0; //-mass(i)*linkcm(i,0)*linkcm(i,1);
        I_parallel(1,1) = Ixyz(i,1) ;//+ mass(i)*(pow(linkcm(i,0),2) + pow(linkcm(i,2),2));
        I_parallel(1,2) = 0; //-mass(i)*linkcm(i,1)*linkcm(i,2);
        I_parallel(2,0) = 0; //- mass(i)*linkcm(i,0)*linkcm(i,2);
        I_parallel(2,1) = 0; //-mass(i)*linkcm(i,1)*linkcm(i,2);
        I_parallel(2,2) = Ixyz(i,2) ;//+ mass(i)*(pow(linkcm(i,0),2) + pow(linkcm(i,1),2));
        // jacobian_cm = get_jacobian_cm(joint_val,i+1);
        T_cm = forward_kine_cm(joint_val, i+1); //use .block
        for(int j=0; j<3; j++)
        {
            for(int k =0; k<3; k++)
            {
                Rg(j,k) = T_cm(j,k);
            }
        }
        Jli = Rg * I_parallel * Rg.transpose();
        Jp.row(0) = get_jacobian_cm(joint_val,i+1).row(0);
        Jp.row(1) = get_jacobian_cm(joint_val,i+1).row(1);
        Jp.row(2) = get_jacobian_cm(joint_val,i+1).row(2);
        Jo.row(0) = get_jacobian_cm(joint_val,i+1).row(3);
        Jo.row(1) = get_jacobian_cm(joint_val,i+1).row(4);
        Jo.row(2) = get_jacobian_cm(joint_val,i+1).row(5);

        // std::cout << "get_jacobian_cm(joint_val,i+1)\n"<< get_jacobian_cm(joint_val,i+1) << "\n" << std::endl;
        // std::cout << "Jp\n"<< Jp << "\n" << std::endl;
        // std::cout << "Jo\n"<< Jo << "\n" << std::endl;

        B += mass(i)*Jp.transpose()*Jp + Jo.transpose()*Jli*Jo;
        // std::cout << "B \n"<< B << "\n" << std::endl;
    }

    return B;
    

}

MatrixXd iiwa14_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{
    //TODO: Fill in this function to complete Q2.
    double delta = 0.00001;
    MatrixXd B1, B2, B_q, test,  Bi(7,7), Bk(7,7);
    VectorXd joint_temp;
    MatrixXd T_cm = Eigen::MatrixXd::Identity(10, 10);
    MatrixXd* B= new MatrixXd[7];
    for (int i; i<7; i++) B[i]=MatrixXd(7,7);
    MatrixXd C = Eigen::MatrixXd::Constant(7, 7, 0);
    

    for(int i=0; i<7; i++)
    {
    	joint_temp = joint_val;
    	B1 = getB(joint_val);
    	// std::cout << "getB(joint_val) \n"<<getB(joint_val) << "\n" << std::endl;
    	joint_temp(i) = joint_temp(i) + delta;
    	B2 =  getB(joint_temp);
    	// std::cout << "B1 \n"<< B1 << "\n" << std::endl;

    	B[i] = (B2 - B1) /delta;
    	// std::cout << "(B2 - B1) /delta \n"<< (B2 - B1) /delta << "\n" << std::endl;
    	// std::cout << "B[i] \n"<< B[i] << "\n" << std::endl;
    	
    }
    for(int i =0; i<7; i++)
    {
    	for(int j =0; j<7; j++)
    	{
    		for(int k=0; k<7; k++)
    		{
    			Bi = B[i];
    			Bk = B[k];
    			C(i,j) += ( Bk(i,j) -0.5*Bi(j,k)  )*joint_vel(k);

    		}

    	}
    }

    // std::cout << "test \n"<< test << "\n" << std::endl;
    // std::cout << " C * vel\n"<< C * joint_vel << "\n" << std::endl;

    
    return C;
    

}

VectorXd iiwa14_kinematic::getG(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
    Vector3d gT;
    gT << 0,0,g;
    VectorXd G(7),Jp(3), temp(7) ;
    for(int i=0; i<7; i++)
    {
    	for(int j=0; j<7; j++)
    	{
    		Jp = get_jacobian_cm(joint_val,j+1).block(0,i,3,1);
    		temp(j) = (mass(j)*gT.transpose()*Jp);
    	}
        G(i) = temp.sum();
    }
    return G;
}


double iiwa14_kinematic::get_psi(Matrix4d pose, VectorXd joint_val)
{
	double psi =0;
    double dbs = DH_params[0][2];
    double dse = DH_params[2][2];
    double dew = DH_params[4][2];
    double dwf = DH_params[6][2];
    
    Vector3d p07;
    Vector3d p26, R01z, p04v, p02v, p06v, p02, p04, p06,p24,p46,p67;
    double theV4, GC6=1, GC4=1, GC2=1, theV1, phi, theV2, theV3=0, theta1, 
    theta2, theta3, theta4, theta5, theta6, theta7, sgpsi;
    VectorXd joint_val_V4(4),joint_val_V3(3),joint_val_V2(2);
    Matrix4d T_V4 = Eigen::Matrix4d::Identity(4, 4);
    Matrix4d T_4 = Eigen::Matrix4d::Identity(4, 4);
    Matrix4d T_V2 = Eigen::Matrix4d::Identity(4, 4);
    Matrix4d T_2 = Eigen::Matrix4d::Identity(4, 4);
    Matrix4d T_6 = Eigen::Matrix4d::Identity(4, 4);
    Vector3d Vvsew,Vsew, offset;
    offset << 0,0,0.1575;
    Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    Matrix3d R = Eigen::Matrix3d::Identity(3, 3);
    Matrix3d p26cross, As, Bs, Cs, Aw, Bw, Cw;
    double sign;


    p02 << 0,0,dbs;
    p24 << 0,dse,0;
    p46 << 0,0,dew;
    p67 << 0,0,dwf;
    // p02 = p02 - offset;
    // p24 = p24 - offset;
    // p46 = p46 - offset;
    // p67 = p67 - offset;


    if(joint_val(1)>=0)
    {
    	GC2 = 1;
    }
    else
    {
    	GC2 = -1;
    }
    if(joint_val(3)>=0)
    {
    	GC4 = 1;
    }
    else
    {
    	GC4 = -1;
    }
    if(joint_val(5)>=0)
    {
    	GC6 = 1;
    }
    else
    {
    	GC6 = -1;
    }

    p07 = pose.block(0,3,3,1) - offset;
    // p07(2) = p07(2) - 0.1575;
    p26 = p07 - p02 - (pose.block(0,0,3,3)*p67);
    theV4 = GC4*acos( ((pow(p26.norm(),2)) - pow(dse,2) - pow(dew,2)  ) / (2*dse*dew)    );
    // std::cout << "theV4\n"<< theV4 << "\n" << std::endl;


 //    std::cout << " forward_kine(joint_val, 1).block(0,0,3,3)\n"<< std::endl;
 //    std::cout <<   forward_kine(joint_val, 1).block(0,2,3,1) << "\n" << std::endl;
 //    std::cout << "p26\n"<< p26 << "\n" << std::endl;

 //    std::cout << "p26.cross( forward_kine(joint_val, 1).block(0,0,3,3))\n"<< "\n" << std::endl;
    R01z = forward_kine(joint_val, 1).block(0,2,3,1);
	// std::cout << p26.cross(R01z) << "\n" << std::endl;


    if(  (p26.cross(R01z)).norm() > 0)
    {
    	theV1 = atan2(p26(1), p26(0));
    }
    else
    {
    	theV1 = 0;
    }
    
    

    phi = acos( (   pow(dse,2) + pow(p26.norm(),2) - pow(dew,2)) /  (2*dse*p26.norm())  );
    theV2 = atan2( pow(  pow(p26(0),2)  + pow(p26(1),2) ,0.5), p26(2) )  + GC4*phi;
    
    joint_val_V4 << theV1,theV2,theV3,theV4;

    T_V4 = forward_kine(joint_val_V4, 4);
    p04v = T_V4.block(0,3,3,1) - offset;
    T_4 = forward_kine(joint_val, 4);
    p04 = T_4.block(0,3,3,1) - offset;

    // joint_val_V2 << theV1,theV2;
    // T_V2 = forward_kine(joint_val_V2, 2);
	// p02v = T_V2.block(0,3,3,1) - offset;
    // T_2 = forward_kine(joint_val, 2);
    // p02 = T_2.block(0,3,3,1) - offset;

    // T_6 = forward_kine(joint_val, 6);
    // p06 = T_6.block(0,3,3,1) - offset;
    // p06v = p02v + p26;
    // p06 = 
    // joint_val_V3 << theV1,theV2,theV3;
    // T_V3 = forward_kine(joint_val_V3, 3);

    Vvsew = (( p04v - p02 )/( (p04v - p02).norm() )).cross( p26/ p26.norm() );
    Vsew = (( p04 - p02 )/( (p04 - p02).norm() )).cross( p26/ p26.norm() );
 	// std::cout << "Vvsew" << "\n" << std::endl;
 	// std::cout << Vvsew << "\n" << std::endl;
 	// std::cout << "Vsew" << "\n" << std::endl;

 	// std::cout << Vsew << "\n" << std::endl;
 	// std::cout << "Vvsew.cross(Vsew)" << "\n" << std::endl;

 	// std::cout << Vvsew.cross(Vsew) << "\n" << std::endl;
 	sgpsi = ( (Vvsew/Vvsew.norm() ).cross(Vsew / Vsew.norm()) ).transpose() * p26;
 	if(sgpsi > 0)
 	{
 		sign = 1;
 	}
 	else
 	{
 		sign = -1;
 	}
 	// std::cout << "sgpsi" << "\n" << std::endl;
 	// std::cout << sgpsi << "\n" << std::endl;
 	psi = sign * acos( (Vvsew/Vvsew.norm()).transpose() * (Vsew / Vsew.norm())   );


    // p26cross << 0, -p26cross(2), p26cross(1),
    //             p26cross(2), 0, -p26cross(0),
    //             -p26cross(1), p26cross(0), 0;


	return psi;
}



