cmake_minimum_required(VERSION 3.0.2)
project(a2_ros2udp)

## Compile as C++11
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  roslib  # for using ros::package

  geometry_msgs
  # controller_manager
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  roslib
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




catkin_install_python(PROGRAMS
  scripts/goal_sender.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



#include_directories(/include)
#link_directories(/lib)

### Go1
include_directories(../../../../sdk/unitree_legged_sdk-v20220117/include)
link_directories(../../../../sdk/unitree_legged_sdk-v20220117/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.so lcm)   # for arm
add_executable(ros_control src/new_ros_control_for_go1_sdk_20220117.cpp)
target_link_libraries(ros_control  ${catkin_LIBRARIES} ${EXTRA_LIBS})


