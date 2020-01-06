cmake_minimum_required(VERSION 2.8.3)
project(rvip)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  rospy
  roscpp
  roslint
  pcl_ros
  std_msgs
  roslaunch
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  visualization_msgs
)

roslint_cpp()

# DEPENDENCIES
find_package(OpenGR REQUIRED PATHS /usr/local/lib/cmake)
find_package(PCL 1.8 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenGR_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS} )

roslaunch_add_file_check(launch)

catkin_package( CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rvip src/main.cpp)
target_link_libraries(rvip ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenGR_LIBRARIES})

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_rvip test/run_test.test src/test/test_rvip.cpp)
  target_link_libraries(test_rvip ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenGR_LIBRARIES})

  # Create a target rvip_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()

endif()
