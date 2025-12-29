cmake_minimum_required(VERSION 2.8.3)
project(plan_ctrl)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf
  roscpp
  rospy
  serial
  std_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plan_ctrl_lib
  CATKIN_DEPENDS visualization_msgs geometry_msgs tf roscpp serial std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(plan_ctrl_lib
  src/L1_controller_v2.cpp
)
target_link_libraries(plan_ctrl_lib ${catkin_LIBRARIES})

add_executable(L1_controller_v2 src/L1_controller_node.cpp)
target_link_libraries(L1_controller_v2 plan_ctrl_lib ${catkin_LIBRARIES})
# add_dependencies(L1_controller_v2_node hypha_racecar_generate_messages_cpp) # This seems to be from a template, maybe not needed if no messages are generated in this package.

add_executable(controller_xcg src/controller_xcg.cpp)
target_link_libraries(controller_xcg ${catkin_LIBRARIES})
# add_dependencies(controller_xcg hypha_racecar_generate_messages_cpp)
