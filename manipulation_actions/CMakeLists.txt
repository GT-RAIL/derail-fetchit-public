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
  message_generation
  moveit_msgs
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  shape_msgs
  std_srvs
  tf_conversions
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(FILES
        BinPick.action
        InHandLocalize.action
        KitManip.action
        LinearMove.action
        StoreObject.action
)

add_message_files(FILES
        ChallengeObject.msg
)

add_service_files(FILES
        AttachArbitraryObject.srv
        AttachSimpleGeometry.srv
        AttachToBase.srv
        ToggleGripperCollisions.srv
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        rail_manipulation_msgs
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

add_executable(cluttered_grasper
        src/ClutteredGrasper.cpp
        )
add_dependencies(cluttered_grasper
        ${PROJECT_NAME}_generate_messages_cpp
        rail_manipulation_msgs_gencpp
        )
target_link_libraries(cluttered_grasper
        ${catkin_LIBRARIES}
        )

add_executable(collision_scene_manager
        src/CollisionSceneManager.cpp
        )
add_dependencies(collision_scene_manager
        ${PROJECT_NAME}_generate_messages_cpp
        rail_manipulation_msgs_gencpp
        )
target_link_libraries(collision_scene_manager
        ${catkin_LIBRARIES}
        )

add_executable(in_hand_localizer
        src/InHandLocalizer.cpp
        )
add_dependencies(in_hand_localizer
        ${PROJECT_NAME}_generate_messages_cpp
        )
target_link_libraries(in_hand_localizer
        ${catkin_LIBRARIES}
        )

add_executable(kit_manipulator
        src/KitManipulator.cpp src/ScoredPose.cpp
        )
add_dependencies(kit_manipulator
        ${PROJECT_NAME}_generate_messages_cpp
        manipulation_actions_gencpp
        )
target_link_libraries(kit_manipulator
        ${catkin_LIBRARIES}
        )

add_executable(linear_controller
        src/LinearController.cpp
        )
add_dependencies(linear_controller
        ${PROJECT_NAME}_generate_messages_cpp
        )
target_link_libraries(linear_controller
        ${catkin_LIBRARIES}
        )

add_executable(schunk_gear_grasper
        src/SchunkGearGrasper.cpp
        )
add_dependencies(schunk_gear_grasper
        ${PROJECT_NAME}_generate_messages_cpp
        manipulation_actions_gencpp
        )
target_link_libraries(schunk_gear_grasper
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS kit_manipulator cluttered_grasper in_hand_localizer schunk_gear_grasper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
