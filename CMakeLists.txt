cmake_minimum_required(VERSION 3.0.2)
project(gps2coorw_axisrot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gps_common
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
)

add_message_files(FILES GPSData.msg)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps2coorw_axisrot
#  CATKIN_DEPENDS geometry_msgs gps_common roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime 
  geometry_msgs
  gps_common
  sensor_msgs
  visualization_msgs
  message_generation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

