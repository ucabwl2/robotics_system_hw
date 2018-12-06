#!/usr/bin/env python

##TODO: Fill in the code. You will have to add your node in the launch file as well. The launch file is in cw3_launch.

import rospy
import numpy as np
import rosbag
import rospkg

camTrobot_gt = np.identity(4)
camTrobot_noisy = np.identity(4)
target_poses = np.zeros((5, 7))

def add_noise_target_pose(pose):
    noisy_pose = np.zeros((5, 7))
    for i in range(0, 5):
        noise_translation = np.random.normal(0, 0.005, 3)
        noise_rotation = np.random.normal(0, 0.0025, 4)

        for j in range(0, 3):
            noisy_pose[i, j] = pose[i, j] + noise_translation[j]

        for j in range(0, 4):
            noisy_pose[i, j + 3] = pose[i, j + 3] + noise_rotation[j]

        norm = np.sqrt(noisy_pose[i, 3]**2 + noisy_pose[i, 4]**2 + noisy_pose[i, 5]**2 + noisy_pose[i, 6]**2)

        for j in range(0, 4):
            noisy_pose[i, j + 3] = noisy_pose[i, j + 3]/norm

    return noisy_pose


def add_noise_obstacles(position):
    noisy_position = [0, 0, 0]
    noise_position = np.random.normal(0, 0.005, 3)
    for i in range(0, 3):
        noisy_position[i] = position[i] + noise_position[i]

    return noisy_position


def main_func():
    rospy.init_node('cw3q3_node', anonymous=True)

    #You can use this to validate your code.
    camTrobot_gt[0, 3] = 0.3
    camTrobot_gt[1, 3] = 0.15

    #You have to use this to actually solve the problem.
    camTrobot_noisy[0, 0] =  np.cos(0.3 * np.pi / 180)
    camTrobot_noisy[0, 1] = -np.sin(0.3 * np.pi / 180)
    camTrobot_noisy[1, 0] =  np.sin(0.3 * np.pi / 180)
    camTrobot_noisy[1, 1] =  np.cos(0.3 * np.pi / 180)
    camTrobot_noisy[0, 3] = 0.302
    camTrobot_noisy[1, 3] = 0.149
    camTrobot_noisy[2, 3] = 0.001

    #These three positions are noise-free, and they are not in the robot coordinate.
    cylinder0_position = [0.3253, 1.0947, 0.4373] #radius = 0.29, length = 0.87
    cylinder1_position = [1.2466, 0.1441, 0.6136] #radius = 0.22, length = 1.23
    box0_position = [0.2577, -0.7061, 0.5784] #size (x, y, z) = (0.380467, 0.530442, 1.156740)

    rospack = rospkg.RosPack()

    path = rospack.get_path('cw3_launch')
    bag = rosbag.Bag(path + '/bags/cw3bag3.bag')

    cc = 0
    for topic, msg, t in bag.read_messages(topics=['target_pose']):
        target_poses[cc, 0] = msg.transform.translation.x
        target_poses[cc, 1] = msg.transform.translation.y
        target_poses[cc, 2] = msg.transform.translation.z
        target_poses[cc, 3] = msg.transform.rotation.w
        target_poses[cc, 4] = msg.transform.rotation.x
        target_poses[cc, 5] = msg.transform.rotation.y
        target_poses[cc, 6] = msg.transform.rotation.z
        cc = cc + 1

    #If you want to validate your algorithm, you can comment these lines. However, you have to uncomment them when you solve the problem.
    target_poses_noisy = add_noise_target_pose(target_poses)
    cylinder0_position_noisy = add_noise_obstacles(cylinder0_position) #No noise is added into its physical dimension.
    cylinder1_position_noisy = add_noise_obstacles(cylinder1_position) #No noise is added into its physical dimension.
    box0_position_noisy = add_noise_obstacles(box0_position) #No noise is added into its physical dimension.


if __name__ == '__main__':
    main_func()