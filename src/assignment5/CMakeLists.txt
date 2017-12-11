cmake_minimum_required(VERSION 2.8.3)
project(assignment5)

add_definitions(-std=c++11)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib COMPONENTS roscpp std_msgs tf)

add_action_files(DIRECTORY action FILES ding.action)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(action_server src/action_server.cpp)
target_link_libraries(action_server ${catkin_LIBRARIES})

add_executable(action_client src/action_client.cpp)
target_link_libraries(action_client ${catkin_LIBRARIES})