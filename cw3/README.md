#CW3_README_file 
#arthor: luke Wenguang

Q2
>> roscore

open a new terminal and run
>> cd ~/catkin_ws

>> catkin_make

>> source devel/setup.bash

>> roslaunch cw3_launch q5a.launch

open a new terminal and run 

>> source devel/setup.bash

>> rosrun cw3q2 iiwa_kine_checker_node

Then we should be able to see 

1.Forward kinematic

2.Forward kinematic at the centre of mass

3.Jacobian

4.Jacobian at the centre of mass

5.Iterative inverse kinematic

6.Closed-form inverse kinematic (the second to last is the same as the input tranformation)

7.B

8.C

9.G

10.C*velocity

11.KDL_B

12.KDL_G

13.KDL_C

Q5a

>> roslaunch cw3_launch q5a.launch

open a new terminal and run

>> rosrun cw3q5 cw3q5a_node

the outputs are the acceleration of each joint

Q5b

>> roslaunch cw3_launch q5b.launch

open a new terminal and run

>> rosrun cw3q5 cw3q5b_node

the outputs are

1.the average mass

2.x position relative to frame 7

3.y position relative to frame 7

4.z position relative to frame 7

5 torque for joint 1

6 torque for joint 2

7 torque for joint 3

8 torque for joint 4

9 torque for joint 5

10 torque for joint 6

11 torque for joint 7


Please wait a few minutes until the process has been finished. 
To get an accurate result, mass and the center of mass is computed until the manipulator reaches the target joint poses.
Because each time the error to the target joint poses are different, the waiting time might vary. 
Also, each time the error of the result might vary.
