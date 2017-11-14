cmake_minimum_required(VERSION 2.8.3)
project(assignment2)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  #message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pid_assignment_2
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pointshoot_node src/pointshoot_node.cpp)
target_link_libraries(pointshoot_node ${catkin_LIBRARIES})

add_executable(servoing_node src/servoing_node.cpp)
add_library(pid_assignment_2 src/pid.cpp)
target_link_libraries(servoing_node pid_assignment_2 ${catkin_LIBRARIES})