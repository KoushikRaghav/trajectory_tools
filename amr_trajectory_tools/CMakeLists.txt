cmake_minimum_required(VERSION 3.0.2)
project(amr_trajectory_tools)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
  tf
  roslib
)

## Find additional required system dependencies
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)

## Generate services
add_service_files(
  FILES
  SaveTrajectory.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_geometry_msgs visualization_msgs geometry_msgs nav_msgs std_msgs message_runtime tf roslib
  DEPENDS EIGEN3 YAML_CPP JSONCPP
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/publish_random_twist.cpp
)

## Add dependencies to the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

## Declare executables
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
add_executable(publish_random_twist src/publish_random_twist.cpp)

## Add dependencies to executables
add_dependencies(trajectory_publisher_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_reader_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link executables
target_link_libraries(trajectory_publisher_saver
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(publish_random_twist
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


target_link_libraries(trajectory_reader_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS ${PROJECT_NAME} trajectory_publisher_saver trajectory_reader_publisher 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)