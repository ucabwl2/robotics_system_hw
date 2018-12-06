#include "cw3q1/iiwa14Kine.h"
#include "random"
#include "chrono"

#include "rosbag/bag.h"
#include "rosbag/view.h"

//TODO: Fill in the code. You will have to add your node in the launch file as well. The launch file is in cw3_launch.

Matrix4d camTrobot_gt, camTrobot_noisy;

Vector3d cylinder0_position, cylinder1_position, box0_position;

MatrixXd target_poses(5, 7);

MatrixXd add_noise_target_pose(MatrixXd A)
{
    MatrixXd A_noise(5, 7);

    for(int i = 0; i < 5; i++)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);

        std::normal_distribution<double> dist1(0.0, 0.005);
        std::normal_distribution<double> dist2(0.0, 0.0025);

        for(int j = 0; j < 3; j++)
            A_noise(i, j) = A(i, j) + dist1(generator);

        for(int j = 3; j < 7;j++)
            A_noise(i, j) = A(i, j) + dist2(generator);

        double norm = sqrt(pow(A_noise(i, 3), 2) + pow(A_noise(i, 4), 2) + pow(A_noise(i, 5), 2) + pow(A_noise(i, 6), 2));
        for (int j = 3; j < 7; j++)
            A_noise(i, j) = A_noise(i, j)/norm;

    }
    return A_noise;
}

Vector3d add_noise_obstacles(Vector3d p)
{
    Vector3d p_noise;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> dist(0.0, 0.005);

    for(int i = 0; i < 3; i++)
        p_noise(i) = p(i) + dist(generator);

    return p_noise;
}

int main (int argc, char **argv)
{

    ros::init(argc, argv, "cw3q3_node");

    //You can use this to validate your code.
    camTrobot_gt << 1, 0, 0, 0.3,
                    0, 1, 0, 0.15,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    //You have to use this to actually solve the problem.
    camTrobot_noisy << cos(0.3*M_PI/180), -sin(0.3*M_PI/180), 0, 0.302,
                        sin(0.3*M_PI/180), cos(0.3*M_PI/180), 0, 0.149,
                        0, 0, 1, 0.001,
                        0, 0, 0, 1;

    //These three positions are noise-free, and they are not in the robot coordinate.
    cylinder0_position << 0.3253, 1.0947, 0.4373; //radius = 0.29, length = 0.87
    cylinder1_position << 1.2466, 0.1441, 0.6136; //radius = 0.22, length = 1.23
    box0_position << 0.2577, -0.7061, 0.5784; //size (x, y, z) = (0.380467, 0.530442, 1.156740)

    rosbag::Bag bag;
    bag.open(MY_BAG);

    int cc = 0;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        geometry_msgs::TransformStamped::ConstPtr i = m.instantiate<geometry_msgs::TransformStamped>();

        if (i != NULL)
        {
            target_poses(cc, 0) = i->transform.translation.x;
            target_poses(cc, 1) = i->transform.translation.y;
            target_poses(cc, 2) = i->transform.translation.z;

            target_poses(cc, 3) = i->transform.rotation.w;
            target_poses(cc, 4) = i->transform.rotation.x;
            target_poses(cc, 5) = i->transform.rotation.y;
            target_poses(cc, 6) = i->transform.rotation.z;

            cc = cc + 1;
        }
    }

    //If you want to validate your algorithm, you can comment these lines. However, you have to uncomment them when you solve the problem.
    MatrixXd target_poses_noisy = add_noise_target_pose(target_poses);
    Vector3d cylinder0_position_noisy = add_noise_obstacles(cylinder0_position); //No noise is added into its physical dimension.
    Vector3d cylinder1_position_noisy = add_noise_obstacles(cylinder1_position); //No noise is added into its physical dimension.
    Vector3d box0_position_noisy = add_noise_obstacles(box0_position); //No noise is added into its physical dimension.


    return 2019;

}
