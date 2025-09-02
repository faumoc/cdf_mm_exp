cmake_minimum_required(VERSION 3.10)
project(mm_ros_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CATKIN_PACKAGE_DEPENDENCIES
    roscpp
    controller_interface
    hardware_interface
    manipulation_msgs
    ddynamic_reconfigure
    tf
    tf2_ros
    tf2_geometry_msgs
    urdf
    kdl_parser
    robot_state_publisher
    visualization_msgs
    geometry_msgs
    mm_msg
    mm_visualize
    interactive_markers)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
# find_library(fmt REQUIRED)

message(STATUS "CRISP include dir: ${CRISP_INCLUDE_DIRS}")
message(STATUS "CRISP library dir: ${CRISP_LIBRARIES}")

# set(ENV{PKG_CONFIG_PATH} "/usr/local/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# Find packages
find_package(PkgConfig REQUIRED)

# CPPAD output the .pc file, find it with pkgconfig
# CPPADCG is header file only, if you install it default, you don't need to find it explicitly
pkg_check_modules(CPPAD REQUIRED cppad)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
system
filesystem
log_setup
log
)
find_package(yaml-cpp REQUIRED) # Yaml for reading the hyper-parameters for the solver
find_package(piqp REQUIRED) # piqp is header only

find_package(casadi CONFIG REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


# set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")

pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
  EIGEN3
  Boost
  pinocchio)
# set(FLAGS
#     ${OCS2_CXX_FLAGS} ${pinocchio_CFLAGS_OTHER} -Wno-ignored-attributes
#     -Wno-invalid-partial-specialization -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#     -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CRISP_INCLUDE_DIRS}                 
  ${EIGEN3_INCLUDE_DIR}     
  ${CPPAD_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)


# link_directories(${pinocchio_LIBRARY_DIRS})
# link_directories(/opt/copt71/lib)
add_library(
  ${PROJECT_NAME}
  src/mm_ros_control.cpp
  src/SwerveTarget.cpp
  src/map_util.cpp
  src/SwerveModelInfo.cpp
)

# add_executable(SDPTEST src/sdptest.cpp) target_link_libraries(SDPTEST
# altro::altro fmt::fmt Eigen3::Eigen ${PCL_LIBRARIES}
# /opt/copt71/lib/libcopt_cpp.so)

# message(WARNING "AltroCpp_LIBRARIES: ${fmt}")
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
         ${Boost_LIBRARIES}
  # fmt::fmt
  # PRIVATE Eigen3::Eigen
  ${PCL_LIBRARIES} 
  ${CPPAD_LIBRARIES}
  yaml-cpp
  piqp::piqp # the piqp library (header file library)
  -ldl
  casadi::casadi # link to casadi
  ${CRISP_LIBRARIES}
  ${pinocchio_LIBRARY_DIRS}
  )

# target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES mm_ros_control_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
