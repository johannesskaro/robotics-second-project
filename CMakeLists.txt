cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs sensor_msgs tf)


## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(convert_odom_to_tf src/convert_odom_to_tf.cpp)
target_link_libraries(convert_odom_to_tf ${catkin_LIBRARIES})
