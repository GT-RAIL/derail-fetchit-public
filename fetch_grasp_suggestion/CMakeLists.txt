cmake_minimum_required(VERSION 2.8.3)
project(fetch_grasp_suggestion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  control_msgs
  eigen_conversions
  geometry_msgs
  interactive_markers
  message_generation
  moveit_msgs
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(FILES
        ExecuteGrasp.action
        ExecuteSelectedGrasp.action
        PresetMove.action
        PresetJointsMove.action
        SuggestGrasps.action
)

add_message_files(
        FILES
        BoundingBox.msg
        GraspFeedback.msg
        RankedGrasp.msg
        RankedGraspList.msg
)

add_service_files(
        FILES
        AddObject.srv
        CycleGrasps.srv
        SuggestGrasps.srv
        ClassifyAll.srv
        ClassifyGraspPair.srv
        PairwiseRank.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
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
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(suggester src/suggester.cpp src/common.cpp src/bounding_box_calculator.cpp)
add_executable(selector src/selector.cpp src/common.cpp src/bounding_box_calculator.cpp)
add_executable(executor src/executor.cpp src/bounding_box_calculator.cpp)
add_executable(test_grasp_suggestion src/test_grasp_suggestion.cpp)
add_executable(cluttered_scene_demo src/cluttered_scene_demo.cpp src/point_cloud_manipulation.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(suggester ${catkin_LIBRARIES} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(selector ${catkin_LIBRARIES})
target_link_libraries(executor ${catkin_LIBRARIES})
target_link_libraries(test_grasp_suggestion ${catkin_LIBRARIES})
target_link_libraries(cluttered_scene_demo ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
add_dependencies(suggester ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(selector ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(executor ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(test_grasp_suggestion ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(cluttered_scene_demo ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS suggester selector executor test_grasp_suggestion cluttered_scene_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
