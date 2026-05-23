cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
  sensor_msgs
  tf
  onboard_detector
  geometry_msgs
  gazebo_msgs
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

catkin_add_env_hooks(99.disable_ros1_eol_warnings
  SHELLS bash
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

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
  ${OpenCV_INCLUDE_DIRS}
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
install(DIRECTORY launch urdf worlds scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.py" EXCLUDE)

catkin_install_python(PROGRAMS
  scripts/pedestrian_sim.py
  scripts/cmd_vel_to_state.py
  scripts/imu_from_state.py
  scripts/steering_bridge.py
  scripts/load_steering_controllers.py
  scripts/registered_map_accumulator.py
  scripts/model_pose_guard.py
  scripts/gazebo_pedestrian_truth_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
