cmake_minimum_required(VERSION 3.0.2)
project(revoMotor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_generation
  roscpp
  rospy
  serial
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  roslib
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  statecmd.msg
  motorinfo.msg
  motorcmd.msg
)

add_service_files(
  FILES
  # motormode.srv
  # resetmode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES revoMotor
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------
# 库 1: revoMotor_info
# ---------------------------------------------------------
add_library(${PROJECT_NAME}_info
  include/revoMotor/revodriver.h
  src/driver/revodriver.cpp
)
# [关键修复] 显式告诉 CMake，编译这个库之前必须先生成消息头文件
add_dependencies(${PROJECT_NAME}_info ${PROJECT_NAME}_generate_messages_cpp)


# ---------------------------------------------------------
# 库 2: revoMotor_test_info
# ---------------------------------------------------------
add_library(${PROJECT_NAME}_test_info
  include/revoMotor/revodriver.h
  src/driver/revodriver_test.cpp
)
# [关键修复] 同上，必须依赖消息生成
add_dependencies(${PROJECT_NAME}_test_info ${PROJECT_NAME}_generate_messages_cpp)


# ---------------------------------------------------------
# 可执行文件 1: revodriver_node
# ---------------------------------------------------------
add_executable(revodriver_node
  src/driver/revodriver.cpp
)

# 这里通常也建议加上 ${PROJECT_NAME}_generate_messages_cpp
add_dependencies(revodriver_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(revodriver_node
  ${PROJECT_NAME}_info
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------
# 可执行文件 2: revodriver_test_node
# ---------------------------------------------------------
add_executable(revodriver_test_node
  src/driver/revodriver_test.cpp
)

# 这里通常也建议加上 ${PROJECT_NAME}_generate_messages_cpp
add_dependencies(revodriver_test_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(revodriver_test_node
  ${PROJECT_NAME}_test_info
  ${catkin_LIBRARIES}
)