cmake_minimum_required(VERSION 2.8.3)
project(omniGKF_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  std_srvs
  message_generation
  # three_dof_planar_manipulator
)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  omniGKFcmd.msg
  omniGKFinfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES omniGKF_control
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SERIAL_INCLUDE_DIRS}
)

# ---------------------------------------------------------
# 可执行文件 1: gkf_test_usb
# ---------------------------------------------------------
add_executable(gkf_test_usb 
  src/test_usb.cpp
  src/omni_gkf_usb.cpp
)
target_link_libraries(gkf_test_usb ${catkin_LIBRARIES} ${SERIAL_LIBRARIES})

# [关键修改] 添加这行，确保编译前先生成消息头文件
add_dependencies(gkf_test_usb ${PROJECT_NAME}_generate_messages_cpp)


# ---------------------------------------------------------
# 可执行文件 2: gkf_server_node
# ---------------------------------------------------------
add_executable(gkf_server_node 
  src/omni_gkf_server.cpp
  src/omni_gkf_usb.cpp
)
target_link_libraries(gkf_server_node ${catkin_LIBRARIES} ${SERIAL_LIBRARIES})

# [关键修改] 添加这行，解决 "omniGKFcmd.h: 没有那个文件" 的错误
add_dependencies(gkf_server_node ${PROJECT_NAME}_generate_messages_cpp)