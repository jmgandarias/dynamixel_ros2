cmake_minimum_required(VERSION 3.5)
project(dynamixel_ros2)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# try to find dynamixel_sdk - if it is not a CMake package you may need to
# link/include it as a system library or provide its include path manually
find_package(dynamixel_sdk REQUIRED OPTIONAL_COMPONENTS)

# Generate ROS 2 messages (adjust filenames to your actual .msg files)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamixelState.msg"
  "msg/DynamixelCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Include dirs
include_directories(
  include
)

# Library target (if you have a shared library source)
# keep or remove depending on whether src/dynamixel_ros2.cpp exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_ros2.cpp")
  add_library(dynamixel_ros2
    src/dynamixel_ros2.cpp
  )
  ament_target_dependencies(dynamixel_ros2
    rclcpp
    std_msgs
  )
endif()

# Executables (convert your old nodes)
add_executable(testCommunication src/testCommunication.cpp)
ament_target_dependencies(testCommunication rclcpp std_msgs)
if(TARGET dynamixel_ros2)
  target_link_libraries(testCommunication dynamixel_ros2)
endif()

add_executable(blinkingLED src/blinkingLED.cpp)
ament_target_dependencies(blinkingLED rclcpp std_msgs)
if(TARGET dynamixel_ros2)
  target_link_libraries(blinkingLED dynamixel_ros2)
endif()

add_executable(testPositionControl src/testPositionControl.cpp)
ament_target_dependencies(testPositionControl rclcpp std_msgs)
if(TARGET dynamixel_ros2)
  target_link_libraries(testPositionControl dynamixel_ros2)
endif()

add_executable(testPWMControl src/testPWMControl.cpp)
ament_target_dependencies(testPWMControl rclcpp std_msgs)
if(TARGET dynamixel_ros2)
  target_link_libraries(testPWMControl dynamixel_ros2)
endif()

add_executable(dmxlParamsMonitor src/dmxlParamsMonitor.cpp)
ament_target_dependencies(dmxlParamsMonitor rclcpp std_msgs)
if(TARGET dynamixel_ros2)
  target_link_libraries(dmxlParamsMonitor dynamixel_ros2)
endif()

add_executable(pinzaTest src/realTest.cpp)
ament_target_dependencies(pinzaTest rclcpp std_msgs)
if(TARGET dynamixel_ros2)
  target_link_libraries(pinzaTest dynamixel_ros2)
endif()

# Install targets
install(TARGETS
  testCommunication
  blinkingLED
  testPositionControl
  testPWMControl
  dmxlParamsMonitor
  pinzaTest
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install message files so they are picked up by rosidl (if you added .msg files)
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
