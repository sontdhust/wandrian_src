cmake_minimum_required(VERSION 2.8.3)
project(wandrian_communicate)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(communicate src/communicate.cpp)
target_link_libraries(communicate ${catkin_LIBRARIES})
add_dependencies(communicate ${PROJECT_NAME}_generate_messages_cpp)
