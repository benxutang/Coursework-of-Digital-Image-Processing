cmake_minimum_required(VERSION 3.0.2)
project(H_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(OpenCV_DIR   /opt/ros/kinetic/share/OpenCV-3.3.1-dev)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spacial_filtering
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
# include
# 链接头文件
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detector 
  src/detector.cpp
  src/H_detect.cpp
  include/H_detector/detector.h
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(detector 
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
