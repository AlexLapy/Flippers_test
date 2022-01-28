cmake_minimum_required(VERSION 3.9)
project(ros_control_example)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package (Threads REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#  FILES
#
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

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
  LIBRARIES ros_control_example
  CATKIN_DEPENDS controller_manager roscpp rospy std_msgs std_srvs message_runtime 
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 src
 include
 include/lib
 include/ctre
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJE   CT_NAME}
#   src/${PROJECT_NAME}/ros_control_example.cpp
# )


set(source src/flippers_node.cpp src/robot_hardware_interface.cpp)

add_executable(${PROJECT_NAME}_node ${source})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
    Threads::Threads
    )


message("Your architecture is " ${CMAKE_LIBRARY_ARCHITECTURE})
message("Your path " ${PROJECT_SOURCE_DIR}/src)
if(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    message("Jetson TX2")
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCore.a)
else()
    message("x86-64")
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCore.a)
endif()

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(PROGRAMS
   scripts/joy_controller
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


