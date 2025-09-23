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
find_package(dynamixel_sdk QUIET)

set(DYNAMIXEL_SDK_FOUND FALSE)
if(dynamixel_sdk_FOUND)
  set(DYNAMIXEL_SDK_FOUND TRUE)
  message(STATUS "Found dynamixel_sdk via find_package")
else()
  # fallback: search for header and library in CMAKE_PREFIX_PATH and common system locations
  find_path(DYNAMIXEL_SDK_INCLUDE_DIR
    NAMES dynamixel_sdk/dynamixel_sdk.h
    PATHS ${CMAKE_PREFIX_PATH} /usr/include /usr/local/include
  )
  find_library(DYNAMIXEL_SDK_LIBRARY
    NAMES dynamixel_sdk dynamixel_sdk_static
    PATHS ${CMAKE_PREFIX_PATH} /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
  )
  if(DYNAMIXEL_SDK_INCLUDE_DIR AND DYNAMIXEL_SDK_LIBRARY)
    message(STATUS "Found dynamixel_sdk headers in ${DYNAMIXEL_SDK_INCLUDE_DIR} and lib ${DYNAMIXEL_SDK_LIBRARY}")
    add_library(dynamixel_sdk::dynamixel_sdk UNKNOWN IMPORTED)
    set_target_properties(dynamixel_sdk::dynamixel_sdk PROPERTIES
      IMPORTED_LOCATION "${DYNAMIXEL_SDK_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${DYNAMIXEL_SDK_INCLUDE_DIR}"
    )
    set(DYNAMIXEL_SDK_FOUND TRUE)
  else()
    message(WARNING "dynamixel_sdk not found. Install ros-$ROS_DISTRO-dynamixel-sdk or build/install the SDK. Compile will fail if code requires the SDK.")
  endif()
endif()

# Guarded generation: only run rosidl_generate_interfaces if msg/*.msg exist
set(ROS_MSG_FILES)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg")
  file(GLOB MSG_FULL_PATHS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
  if(MSG_FULL_PATHS)
    foreach(_m IN LISTS MSG_FULL_PATHS)
      list(APPEND ROS_MSG_FILES "msg/${_m}")
    endforeach()
  endif()
endif()

if(ROS_MSG_FILES)
  message(STATUS "ROS messages found: ${ROS_MSG_FILES}")
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${ROS_MSG_FILES}
    DEPENDENCIES std_msgs
  )
  ament_export_dependencies(rosidl_default_runtime)
else()
  message(STATUS "No .msg files found in ${CMAKE_CURRENT_SOURCE_DIR}/msg — skipping rosidl_generate_interfaces")
endif()
# --- end guarded generation ---

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

# after creating executables / libraries, link the SDK only if found
# example for testCommunication and other targets:
if(DYNAMIXEL_SDK_FOUND)
  if(TARGET dynamixel_sdk::dynamixel_sdk)
    # use plain signature to match ament_target_dependencies usage
    target_link_libraries(testCommunication dynamixel_sdk::dynamixel_sdk)
    target_link_libraries(blinkingLED dynamixel_sdk::dynamixel_sdk)
    target_link_libraries(testPositionControl dynamixel_sdk::dynamixel_sdk)
    target_link_libraries(testPWMControl dynamixel_sdk::dynamixel_sdk)
    target_link_libraries(dmxlParamsMonitor dynamixel_sdk::dynamixel_sdk)
    target_link_libraries(pinzaTest dynamixel_sdk::dynamixel_sdk)
    if(TARGET dynamixel_ros2)
      target_link_libraries(dynamixel_ros2 dynamixel_sdk::dynamixel_sdk)
    endif()
  endif()
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
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg")
  install(DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(STATUS "No msg/ directory found — skipping install(DIRECTORY msg ...)")
endif()

ament_package()
