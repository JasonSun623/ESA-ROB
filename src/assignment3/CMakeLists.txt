cmake_minimum_required(VERSION 2.8.3)
project(assignment3)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  #message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pid
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(follow_carrot_node src/follow_carrot_node.cpp)
add_library(pid src/pid.cpp)
target_link_libraries(follow_carrot_node pid ${catkin_LIBRARIES})

add_executable(rectangle_planner_node src/rectangle_planner_node.cpp)
target_link_libraries(rectangle_planner_node ${catkin_LIBRARIES})

add_executable(triangle_planner_node src/triangle_planner_node.cpp)
target_link_libraries(triangle_planner_node ${catkin_LIBRARIES})

