cmake_minimum_required(VERSION 2.8.3)
project(rail_object_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  message_generation
)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

add_message_files(
   FILES
   Descriptor.msg
)

add_service_files(
   FILES
   ExtractPointCloud.srv
   PartsQuery.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)


catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime)

include_directories(include
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(LINK_LIBS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME} src/ObjectRecognition.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
