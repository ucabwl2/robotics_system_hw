cmake_minimum_required(VERSION 2.8.3)
project(cw3q2)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  kdl_parser
  tf2_ros
)

find_package(Eigen3 REQUIRED)


set(HEADER_FILES include/cw3q2/iiwa14Kine.h)
set(HEADER_FILES include/cw3q2/kdl_kine_solver.h)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES cw3q2
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2_ros rospy
        DEPENDS EIGEN3 orocos_kdl kdl_parser
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
##add this line to use iiwa_kine_checker_node
add_executable(iiwa_kine_checker_node src/iiwa_kine_checker_node.cpp src/kdl_kine_solver.cpp src/iiwa14Kine.cpp)
##add this line to use iiwa_kine_checker_node
target_link_libraries(iiwa_kine_checker_node
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
        src/iiwa14Kine.cpp
	src/kdl_kine_solver.cpp
        ${HEADER_FILES}
)

##add this line to use DH_param_checker.cpp
add_executable(DH_param_checker src/DH_param_checker.cpp src/iiwa14Kine.cpp)
##add this line to use DH_param_checker.cpp
target_link_libraries(DH_param_checker
  ${catkin_LIBRARIES}
)



