cmake_minimum_required(VERSION 2.8.3)
project(data_recorder)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  actionlib
  actionlib_msgs
)

catkin_python_setup()

add_action_files(
   FILES
   Playback.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
