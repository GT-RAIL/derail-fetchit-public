cmake_minimum_required(VERSION 2.8.3)
project(manipulation_actions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  eigen_conversions
  geometry_msgs
  interactive_markers
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  std_srvs
  tf_conversions
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

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

add_executable(cluttered_grasper
        src/ClutteredGrasper.cpp
        )
add_dependencies(cluttered_grasper
        rail_manipulation_msgs_gencpp
        )
target_link_libraries(cluttered_grasper
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cluttered_grasper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
