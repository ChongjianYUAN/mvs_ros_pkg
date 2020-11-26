cmake_minimum_required(VERSION 3.0.2)
project(mvs_ros_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
  lib
)

add_executable(grabImg src/grab.cpp)
add_executable(grabImgWithTrigger src/grab.cpp)

target_link_libraries(
  grabImg
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lMvCameraControl
  -lpthread
)
target_link_libraries(
  grabImgWithTrigger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lMvCameraControl
  -lpthread
)


