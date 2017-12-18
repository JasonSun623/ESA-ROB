cmake_minimum_required(VERSION 2.8.3)
project(assignment6)

add_definitions(-std=c++11)

find_package(catkin REQUIRED genmsg actionlib COMPONENTS roscpp std_msgs)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(assignment6 src/move_points.cpp)
target_link_libraries(assignment6 ${catkin_LIBRARIES})
