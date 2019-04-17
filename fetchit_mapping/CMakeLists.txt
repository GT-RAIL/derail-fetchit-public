cmake_minimum_required(VERSION 2.8.3)
project(fetchit_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  roslib
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  tf
  tf2_ros
  laser_geometry
  sensor_msgs
  message_filters
)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(challenge_mapping_node src/challenge_mapping_node.cpp)
add_dependencies(challenge_mapping_node ${catkin_EXPORTED_TARGETS})

add_executable(point_filter_node src/point_filter_node.cpp src/PointFilters.cpp)
add_dependencies(point_filter_node ${catkin_EXPORTED_TARGETS})

add_executable(fetch_goal_publisher src/goal_publisher.cpp)
add_dependencies(fetch_goal_publisher ${catkin_EXPORTED_TARGETS})

add_executable(fetch_go_to_goal src/go_to_goal.cpp)
add_dependencies(fetch_go_to_goal ${catkin_EXPORTED_TARGETS})

target_link_libraries(challenge_mapping_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(point_filter_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(fetch_goal_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(fetch_go_to_goal
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS bin/waypoint_publisher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS challenge_mapping_node point_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
