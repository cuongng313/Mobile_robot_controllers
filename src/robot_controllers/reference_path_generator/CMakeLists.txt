cmake_minimum_required(VERSION 3.0.2)
project(reference_path_generator)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(reference_path_generator src/ref_path_generator.cpp)
target_link_libraries(reference_path_generator ${catkin_LIBRARIES})
