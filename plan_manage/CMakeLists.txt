cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
  tf
  onboard_detector
  geometry_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATIKIN_DEPENDS plan_env path_searching bspline bspline_opt
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include  # 确保包含生成的消息头文件
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# replan FSM
add_executable(kin_replan_node
		src/kin_replan_node.cpp
    src/planner_manager.cpp
)
add_dependencies(kin_replan_node ${catkin_EXPORTED_TARGETS} onboard_detector_generate_messages_cpp)
target_link_libraries(kin_replan_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# sim: removed — use kin_replan_node for both simulation and real

# Python scripts
catkin_install_python(PROGRAMS
  scripts/pedestrian_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)