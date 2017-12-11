cmake_minimum_required(VERSION 2.8.3)
project(assignment5)

add_definitions(-std=c++11)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib COMPONENTS roscpp std_msgs tf)

# add_action_files(DIRECTORY action FILES TurtleMove.action)

add_message_files(DIRECTORY msg FILES Triangle.msg)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(move_triangle src/move_triangle.cpp)
target_link_libraries(move_triangle ${catkin_LIBRARIES})
