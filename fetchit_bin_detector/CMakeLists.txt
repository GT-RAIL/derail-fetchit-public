cmake_minimum_required(VERSION 2.8.3)
project(fetchit_bin_detector)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(PACKAGE_DEPENDENCIES
  roscpp
  rail_manipulation_msgs
  manipulation_actions
  visualization_msgs
  asr_approx_mvbb
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

pkg_check_modules( gsl REQUIRED gsl )

add_service_files(
  DIRECTORY srv
  FILES
  GetBinPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${gsl_INCLUDE_DIRS}
    ${asr_approx_mvbb_INCLUDE_DIRS}
)

set(LINK_LIBS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${gsl_LIBRARIES}
    ${asr_approx_mvbb_LIBRARIES}
)

# compile cpp classes
add_library(${PROJECT_NAME} src/BinDetector.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
    rail_manipulation_msgs_generate_messages_cpp manipulation_actions_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

# compiles cpp node
add_executable(bin_detector_node src/bin_detector_node.cpp)
add_dependencies(bin_detector_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
        manipulation_actions_generate_messages_cpp)
target_link_libraries(bin_detector_node ${PROJECT_NAME} ${LINK_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} bin_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
