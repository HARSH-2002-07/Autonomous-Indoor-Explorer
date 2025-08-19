cmake_minimum_required(VERSION 3.0.2)
project(autonomous_explorer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/obstacle_avoidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
