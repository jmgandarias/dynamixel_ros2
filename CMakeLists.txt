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
# try to find dynamixel_sdk (robust: prefer Config package, fallback to headers/libs)
find_package(dynamixel_sdk REQUIRED)

# <-- Add this so targets can include headers from the package's include/ while building
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# instalar headers para que otros paquetes los encuentren
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
  ament_export_include_directories(include)
endif()

# Si tienes ejecutables que usan dynamixel_sdk, enlázalos también
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/testCommunication.cpp")
  add_executable(testCommunication src/testCommunication.cpp)
  ament_target_dependencies(testCommunication rclcpp)
  # link against dynamixel_sdk if the find_package provides a target or variables
  if(TARGET dynamixel_sdk::dynamixel_sdk)
    target_link_libraries(testCommunication dynamixel_sdk::dynamixel_sdk)
  else()
    if(DEFINED dynamixel_sdk_LIBRARIES)
      target_link_libraries(testCommunication ${dynamixel_sdk_LIBRARIES})
    endif()
    if(DEFINED dynamixel_sdk_INCLUDE_DIRS)
      target_include_directories(testCommunication PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
    endif()
  endif()
endif()

# Build a library from the implementation so executables can link to it
# (dynamixel_ros2.cpp contains the dynamixelMotor implementation)
add_library(dynamixel_ros2_lib SHARED src/dynamixel_ros2.cpp)
target_include_directories(dynamixel_ros2_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dynamixel_ros2_lib rclcpp std_msgs)
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(dynamixel_ros2_lib dynamixel_sdk::dynamixel_sdk)
endif()

# Executables (convert your old nodes)
add_executable(blinkingLED src/blinkingLED.cpp)
ament_target_dependencies(blinkingLED rclcpp std_msgs)
# link to local library implementing dynamixelMotor
target_link_libraries(blinkingLED dynamixel_ros2_lib)

add_executable(testPositionControl src/testPositionControl.cpp)
ament_target_dependencies(testPositionControl rclcpp std_msgs)
target_link_libraries(testPositionControl dynamixel_ros2_lib)

add_executable(testPWMControl src/testPWMControl.cpp)
ament_target_dependencies(testPWMControl rclcpp std_msgs)
target_link_libraries(testPWMControl dynamixel_ros2_lib)

add_executable(dmxlParamsMonitor src/dmxlParamsMonitor.cpp)
ament_target_dependencies(dmxlParamsMonitor rclcpp std_msgs)
target_link_libraries(dmxlParamsMonitor dynamixel_ros2_lib)

add_executable(pinzaTest src/realTest.cpp)
ament_target_dependencies(pinzaTest rclcpp std_msgs)
target_link_libraries(pinzaTest dynamixel_ros2_lib)

# if testCommunication exists, link it too (it may be created earlier)
if(TARGET testCommunication)
  target_link_libraries(testCommunication dynamixel_ros2_lib)
endif()

# link the SDK to executables (keep this after library/executables are defined)
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(blinkingLED dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(testPositionControl dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(testPWMControl dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(dmxlParamsMonitor dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(pinzaTest dynamixel_sdk::dynamixel_sdk)
  if(TARGET testCommunication)
    target_link_libraries(testCommunication dynamixel_sdk::dynamixel_sdk)
  endif()
endif()

# Install the library and export target
install(TARGETS dynamixel_ros2_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg")
  install(DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(STATUS "No msg/ directory found — skipping install(DIRECTORY msg ...)")
endif()

ament_package()
