cmake_minimum_required(VERSION 2.8.3)
project(cw3q4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cw3q1
  rosbag
  trajectory_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cw3q4a_node src/cw3q4a.cpp)
add_executable(cw3q4b_node src/cw3q4b.cpp)
add_executable(cw3q4c_node src/cw3q4c.cpp)

add_dependencies(cw3q4a_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cw3q4b_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cw3q4c_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cw3q4a_node
  ${catkin_LIBRARIES}
)

target_link_libraries(cw3q4b_node
  ${catkin_LIBRARIES}
)

target_link_libraries(cw3q4c_node
  ${catkin_LIBRARIES}
)
