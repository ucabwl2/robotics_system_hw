cmake_minimum_required(VERSION 2.8.3)
project(cw3q3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cw3q1
  rosbag
  trajectory_msgs
)

add_definitions(-DMY_BAG="${PROJECT_SOURCE_DIR}/../cw3_launch/bags/cw3bag3.bag")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cw3q3_node src/cw3q3.cpp)

add_dependencies(cw3q3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cw3q3_node
  ${catkin_LIBRARIES}
)
