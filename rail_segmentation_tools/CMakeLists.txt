cmake_minimum_required(VERSION 2.8.3)
project(rail_segmentation_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rail_manipulation_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(merger
  src/Merger.cpp
)
add_executable(tester
  src/Tester.cpp
)

## Add message build dependencies (needed for source build)
add_dependencies(merger
  rail_manipulation_msgs_generate_messages_cpp
)
add_dependencies(tester
  rail_manipulation_msgs_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(merger
  ${catkin_LIBRARIES}
)
target_link_libraries(tester
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS merger tester
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
