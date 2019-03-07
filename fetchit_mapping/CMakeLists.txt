cmake_minimum_required(VERSION 2.8.3)
project(fetchit_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(PACKAGE_DEPENDENCIES
  roscpp
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  tf
)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(challenge_mapping_node src/challenge_mapping_node.cpp)
add_dependencies(challenge_mapping_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(challenge_mapping_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS challenge_mapping_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)