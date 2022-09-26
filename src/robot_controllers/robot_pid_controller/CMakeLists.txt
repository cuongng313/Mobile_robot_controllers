cmake_minimum_required(VERSION 3.0.2)
project(robot_pid_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_controller src/robot_controllers_PID.cpp src/PID.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})