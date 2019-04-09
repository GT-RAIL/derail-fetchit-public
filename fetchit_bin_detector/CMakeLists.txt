cmake_minimum_required(VERSION 2.8.3)
project(fetchit_bin_detector)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(PACKAGE_DEPENDENCIES
  roscpp
  rail_manipulation_msgs
  visualization_msgs
  asr_approx_mvbb
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  geometry_msgs
)

find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

pkg_check_modules( gsl REQUIRED gsl )

catkin_package(
    #INCLUDE_DIRS include
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${gsl_INCLUDE_DIRS}
    ${asr_approx_mvbb_INCLUDE_DIRS}
)

add_executable(bin_detector_node src/bin_detector_node.cpp)
add_dependencies(bin_detector_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(bin_detector_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${gsl_LIBRARIES}
    ${asr_approx_mvbb_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS bin_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)