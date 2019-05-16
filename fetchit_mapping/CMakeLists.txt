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
  nav_msgs
  tf
  tf2_ros
  laser_geometry
  sensor_msgs
  message_filters
  message_generation
)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES Reposition.action
  FILES Navigation.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(challenge_mapping_node src/challenge_mapping_node.cpp)
add_dependencies(challenge_mapping_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(point_filter_node src/point_filter_node.cpp src/PointFilters.cpp)
add_dependencies(point_filter_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fetch_goal_publisher src/goal_publisher.cpp)
add_dependencies(fetch_goal_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fetch_go_to_goal src/go_to_goal.cpp)
add_dependencies(fetch_go_to_goal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(reposition_client src/reposition_client.cpp)
add_dependencies(reposition_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(reposition_server src/reposition_server.cpp)
add_dependencies(reposition_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(navigation_server src/navigation_server.cpp)
add_dependencies(navigation_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(navigation_client src/navigation_client.cpp)
add_dependencies(navigation_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

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

target_link_libraries(reposition_server
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

target_link_libraries(reposition_client
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

target_link_libraries(navigation_server
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

target_link_libraries(navigation_client
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/waypoint_publisher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS challenge_mapping_node point_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
