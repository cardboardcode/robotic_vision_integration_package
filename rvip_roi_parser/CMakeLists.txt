cmake_minimum_required(VERSION 2.8.3)
project(rvip_roi_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  darknet_ros_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rvip_roi_parser src/main.cpp)
target_link_libraries(rvip_roi_parser ${catkin_LIBRARIES})
