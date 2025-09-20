cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Package dependencies
set(${PROJECT_NAME}_DEPENDS
  roscpp
  std_msgs
  message_generation
  message_runtime
  dynamixel_sdk
)

## Include folders
# set(DYN_SDK_DIR ../DynamixelSDK)
# set(${PROJECT_NAME}_INCLUDE_DIRS
#   include
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/include 
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/include/dynamixel_sdk
# )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

## Export (include dirs, libraries, etc)
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS} 
  # ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  # ${${PROJECT_NAME}_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dynamixel_ros
  src/dynamixel_ros.cpp
)

# add_library(dynamixel_sdk
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/packet_handler.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/protocol1_packet_handler.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/protocol2_packet_handler.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/group_sync_read.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/group_sync_write.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/group_bulk_read.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/group_bulk_write.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/port_handler.cpp
#   ${DYN_SDK_DIR}/ros/dynamixel_sdk/src/dynamixel_sdk/port_handler_linux.cpp
# )

## Example Nodes

# Test communication node
add_executable(testCommunication src/testCommunication.cpp)
add_dependencies(testCommunication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testCommunication
  dynamixel_ros
  ${catkin_LIBRARIES}
)

# Blinking LED node
add_executable(blinkingLED src/blinkingLED.cpp)
add_dependencies(blinkingLED ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blinkingLED
  dynamixel_ros
  ${catkin_LIBRARIES}
)

# Test position control node
add_executable(testPositionControl src/testPositionControl.cpp)
add_dependencies(testPositionControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testPositionControl
  dynamixel_ros
  ${catkin_LIBRARIES}
)

# Test PWM control node
add_executable(testPWMControl src/testPWMControl.cpp)
add_dependencies(testPWMControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testPWMControl
  dynamixel_ros
  ${catkin_LIBRARIES}
)

# Dynamixel Parameters monitor norde
add_executable(dmxlParamsMonitor src/dmxlParamsMonitor.cpp)
add_dependencies(dmxlParamsMonitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dmxlParamsMonitor
  dynamixel_ros
  ${catkin_LIBRARIES}
)

add_executable(pinzaTest src/realTest.cpp)
add_dependencies(pinzaTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pinzaTest
  dynamixel_ros
  ${catkin_LIBRARIES}
)
