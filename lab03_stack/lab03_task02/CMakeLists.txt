cmake_minimum_required(VERSION 2.8.3)
project(lab03_task02)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_pub_node src/lab03_task02_pub_node.cpp)
add_executable(${PROJECT_NAME}_sub_node src/lab03_task02_sub_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_pub_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_sub_node
        ${catkin_LIBRARIES}
        )
