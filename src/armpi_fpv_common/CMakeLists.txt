cmake_minimum_required(VERSION 3.0.2)

project(armpi_fpv_common)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hiwonder_sdk 
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hiwonder_sdk 
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
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
