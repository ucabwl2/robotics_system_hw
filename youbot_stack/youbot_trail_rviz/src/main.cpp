#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>

#include <boost/foreach.hpp>
#include <Eigen/Dense>

#define foreach BOOST_FOREACH

using namespace Eigen;

visualization_msgs::Marker reached_points, robot_trail, points;
geometry_msgs::Point point_buffer;
double d;

double dh_a[5] = {0.033, 0.155, 0.135, 0.0, 0.0};
double dh_d[5] = {0.147, 0.0, 0.0, 0.0, 0.183};
double dh_alpha[5] = {M_PI_2, 0.0, 0.0, M_PI_2, 0.0};
double dh_theta[5] = {170*M_PI/180, 65*M_PI/180 + M_PI_2, -146*M_PI/180, 102.5*M_PI/180 + M_PI_2, 167.5*M_PI/180 + M_PI};
double joint[5];

Matrix4d manual_forward_kinematics(double joint[]) {
    Matrix4d A = MatrixXd::Identity(4, 4);
    Matrix4d T, Ti;
    Ti(3, 0) = 0.0;
    Ti(3, 1) = 0.0;
    Ti(3, 2) = 0.0;
    Ti(3, 3) = 1.0;
    Ti(2, 0) = 0.0;
    for (int i = 0; i < 5; i++)
    {
        double input_theta = dh_theta[i] + joint[i];
        Ti(0, 0) = cos(input_theta);
        Ti(1, 0) = sin(input_theta);
        Ti(0, 1) = -sin(input_theta)*cos(dh_alpha[i]);
        Ti(1, 1) = cos(input_theta)*cos(dh_alpha[i]);
        Ti(2, 1) = sin(dh_alpha[i]);
        Ti(0, 2) = sin(input_theta)*sin(dh_alpha[i]);
        Ti(1, 2) = -cos(input_theta)*sin(dh_alpha[i]);
        Ti(2, 2) = cos(dh_alpha[i]);
        Ti(0, 3) = dh_a[i]*cos(input_theta);
        Ti(1, 3) = dh_a[i]*sin(input_theta);
        Ti(2, 3) = dh_d[i];

        T = Ti * A;
    }

    return A;

}

void update_line(const gazebo_msgs::LinkStates::ConstPtr &pos)
{
    int L = pos->pose.size();

    point_buffer.x = 0.5*(pos->pose.at(L - 1).position.x + pos->pose.at(L - 2).position.x);
    point_buffer.y = 0.5*(pos->pose.at(L - 1).position.y + pos->pose.at(L - 2).position.y);
    point_buffer.z = 0.5*(pos->pose.at(L - 1).position.z + pos->pose.at(L - 2).position.z);

    if (robot_trail.points.size() < 800)
    {
        robot_trail.points.push_back(point_buffer);
    }
    else
    {
        robot_trail.points.erase(robot_trail.points.begin());
        robot_trail.points.push_back(point_buffer);
    }

}

void update_point()
{
    for (int i = 0; i < robot_trail.points.size(); i++)
    {
        for (int j = 0; j < points.points.size(); j++)
        {
            d = sqrt(pow(robot_trail.points.at(i).x - points.points.at(j).x, 2) +
                             pow(robot_trail.points.at(i).y - points.points.at(j).y, 2) +
                             pow(robot_trail.points.at(i).z - points.points.at(j).z, 2));
            if (d < 0.02)
            {
                reached_points.points.push_back(points.points.at(j));
                points.points.erase(points.points.begin() + j);
                return;
            }
        }
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(30);

    int checkpoint_data = atoi(argv[1]);

    //Define points message
    points.header.frame_id = robot_trail.header.frame_id = reached_points.header.frame_id = "/base_link";
    points.header.stamp = robot_trail.header.stamp = reached_points.header.stamp = ros::Time::now();
    points.ns = robot_trail.ns = reached_points.ns = "points_and_lines";
    points.action = robot_trail.action = reached_points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = robot_trail.pose.orientation.w = reached_points.pose.orientation.w = 1.0;

    points.id = 0;
    robot_trail.id = 1;
    reached_points.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    reached_points.type = visualization_msgs::Marker::POINTS;
    robot_trail.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    // Unreached points are red
    points.color.r = 1.0f;
    points.color.a = 1.0;


    // POINTS markers use x and y scale for width/height respectively
    reached_points.scale.x = 0.01;
    reached_points.scale.y = 0.01;

    // Reached points are red
    reached_points.color.g = 1.0f;
    reached_points.color.a = 1.0;


    //Define the width of robot trail
    robot_trail.scale.x = 0.005;

    // Robot trails are white
    robot_trail.color.r = 1.0f;
    robot_trail.color.g = 1.0f;
    robot_trail.color.b = 1.0f;
    robot_trail.color.a = 1.0;

    rosbag::Bag bag;

    switch (checkpoint_data)
    {
        case 1: bag.open(MY_BAG_PATH1, rosbag::bagmode::Read); break;
        case 2: bag.open(MY_BAG_PATH2, rosbag::bagmode::Read); break;
        case 3: bag.open(MY_BAG_PATH3, rosbag::bagmode::Read); break;
        case 4: bag.open(MY_BAG_PATH4, rosbag::bagmode::Read); break;
        case 5: bag.open(MY_BAG_PATH5, rosbag::bagmode::Read); break;
        default: ROS_ERROR("Invalid input."); return 0; break;
    }

    std::vector<std::string> topics;

    if (checkpoint_data == 1)
    {
        topics.push_back(std::string("joint_data"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::JointState::ConstPtr s = m.instantiate<sensor_msgs::JointState>();
            if (s != NULL)
            {

                for (int i = 0; i < 5; i++)
                    joint[i] = s->position.at(i);

                Matrix4d current_pose = manual_forward_kinematics(joint);

                geometry_msgs::Point p;
                p.x = current_pose(0, 3);
                p.y = current_pose(1, 3);
                p.z = current_pose(2, 3);
                points.points.push_back(p);
            }

        }
    }
    else if (checkpoint_data == 2)
    {
        topics.push_back(std::string("target_tf"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::TransformStamped::ConstPtr s = m.instantiate<geometry_msgs::TransformStamped>();
            if (s != NULL)
            {
                geometry_msgs::Point p;
                p.x = s->transform.translation.x;
                p.y = s->transform.translation.y;
                p.z = s->transform.translation.z;
                points.points.push_back(p);
            }

        }
    }
    else
    {
        topics.push_back(std::string("target_position"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::Point::ConstPtr s = m.instantiate<geometry_msgs::Point>();
            if (s != NULL)
            {
                geometry_msgs::Point p;
                p.x = s->x;
                p.y = s->y;
                p.z = s->z;
                points.points.push_back(p);
            }

        }
    }

    bag.close();

    while (ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(robot_trail);
        marker_pub.publish(reached_points);

        update_point();

        ros::spinOnce();

        r.sleep();
    }
}
