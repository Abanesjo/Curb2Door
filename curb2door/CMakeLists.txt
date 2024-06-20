cmake_minimum_required(VERSION 3.0.2)
project(curb2door)
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES curb2door
#  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

#
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)