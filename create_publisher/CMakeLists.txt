# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(create_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build communicate
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(communicate src/communicate.cpp)
target_link_libraries(communicate ${catkin_LIBRARIES})
add_dependencies(communicate create_publisher_generate_messages_cpp)

# %EndTag(FULLTEXT)%
