cmake_minimum_required(VERSION 2.8.3)
project(youbot_kinematic)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf2_ros
  tf
)

find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
find_package(orocos_kdl REQUIRED)

set(HEADER_FILES
		include/youbot_kinematic/YoubotKDL.h
		include/youbot_kinematic/YoubotKinematics.h)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES youbot_kinematic
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2_ros tf
  DEPENDS orocos_kdl EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  $ENV{EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
	src/YoubotKinematics.cpp
	src/YoubotKDL.cpp
	${HEADER_FILES}
)
