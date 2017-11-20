cmake_minimum_required(VERSION 2.8.3)
project(assignment4)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})

add_executable(robot_state_publisher src/robot_state_publisher.cpp)
target_link_libraries(robot_state_publisher ${catkin_LIBRARIES})