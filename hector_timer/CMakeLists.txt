cmake_minimum_required(VERSION 3.0.2)
project(hector_timer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hector_timer_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_timer
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)




add_library(hector_timer
  src/hector_timer.cpp
)

target_link_libraries( hector_timer
${catkin_LIBRARIES}
)

add_executable(
  hector_timer_node    src/hector_timer_node.cpp
)


 target_link_libraries(hector_timer_node
  #  ${catkin_LIBRARIES}
  hector_timer
 )

