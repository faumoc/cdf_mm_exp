cmake_minimum_required(VERSION 3.5)
project(mm_visualize)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  visualization_msgs
)
find_package(pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mm_config
  CATKIN_DEPENDS roscpp rospy roslib std_msgs visualization_msgs
  DEPENDS pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mm_config
  src/mm_config.cpp
)

target_link_libraries(mm_config
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS mm_config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
