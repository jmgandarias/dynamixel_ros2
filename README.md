# dynamixel_ros2

Library to use Dynamixel motors in ROS 2. This package provides a small wrapper library and headers to simplify using Dynamixel SDK-based motors from other ROS 2 packages.

This repository is based on the original [dynamixel_ros](https://github.com/TaISLab/dynamixel_ros) and is intended for Ubuntu 22.04 with ROS 2 Humble.

## Contents

- Headers: include/dynamixel_ros2.h
- Library: builds a shared library (dynamixel_ros2_lib)
- Example nodes: simple test programs under src/

## Requirements

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) package:
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-dynamixel-sdk
```

## Build

From the workspace root:
```bash
colcon build --packages-select dynamixel_ros2
# then (important) source the overlay:
source install/setup.bash
```

If you plan to build your package in the same workspace, build `dynamixel_ros2` first or build the whole workspace and then source the install overlay.

## Using this library from another ROS 2 package

Two common setups:

A) You depend on the installed package (recommended if `dynamixel_ros2` is installed system-wide or present in the workspace install overlay)

1. package.xml
Add build / exec dependencies:
```xml
<build_depend>dynamixel_ros2</build_depend>
<exec_depend>dynamixel_ros2</exec_depend>
<build_depend>dynamixel_sdk</build_depend>
<exec_depend>dynamixel_sdk</exec_depend>
```

2. CMakeLists.txt
Find the package, add ament dependencies and link against the library/target.

Minimal example (compatible with exported target or plain library name):

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_ros2 REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs dynamixel_ros2 dynamixel_sdk)

# two ways to link (try exported target first, fallback to plain name)
if(TARGET dynamixel_ros2_lib)
  target_link_libraries(my_node dynamixel_ros2_lib)
else()
  target_link_libraries(my_node dynamixel_ros2_lib)  # plain-name fallback — works if library is in prefix/lib
endif()
```

Then build your package (make sure to source the overlay that contains dynamixel_ros2):
```bash
source /path/to/workspace/install/setup.bash
colcon build --packages-select my_package
```

B) Building both packages in the same workspace
Place dynamixel_ros2 and your package in the same workspace (src/...), then:
```bash
colcon build
source install/setup.bash
```
CMake will find dynamixel_ros2 from the workspace overlay.

## Example usage (C++)

Include the header and use the provided class:

```cpp
#include <dynamixel_ros2.h>

int main()
{
  // construct motor object (example api — adapt to your header)
  dynamixelMotor motor("/dev/ttyUSB0", 1);

  // initialize communication (port, baudrate, protocol version)
  motor.iniComm("/dev/ttyUSB0", 57600.0f, 1);

  motor.setTorqueState(true);
  motor.setOperatingMode(3); // example mode
  motor.setGoalPosition(512); // example command

  // read status
  double pos = motor.getPresentPosition();
  double vel = motor.getPresentVelocity();
  double cur = motor.getPresentCurrent();

  return 0;
}
```

Refer to the header (include/dynamixel_ros2.h) for the exact API and constructors available.

## Troubleshooting

- Linker errors (undefined references)
  - Make sure you sourced the workspace install overlay: source install/setup.bash
  - Verify the library file exists in the overlay: `ls -l install/dynamixel_ros2/lib` or `/opt/ros/$ROS_DISTRO/lib`
  - If you get "Could not find dynamixel_ros2 library/target" warnings, ensure dynamixel_ros2 was built and installed into the overlay you sourced.

- AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH warnings
  - These usually mean an overlay path is stale. Re-source the correct install/setup.bash and/or clear stale environment entries.

## Notes

- This package depends on the official Dynamixel SDK ROS package; install it via apt as shown above.
- If you modify the library (CMake target name or export behavior), update external packages accordingly (they must link to the exported target or library name you provide).

## License

This project is licensed under the GNU General Public License v3 (GPL-3.0). See the included LICENSE file for full text. When distributing derived works, follow the obligations in GPLv3 (preserve notices, provide source, etc.).

## Contact
- Author: Juan M. Gandarias
- Email: jmgandarias@uma.es
- Web: jmgandarias.com

## Notes
- If you change the library target name or export behavior in this package, update dependent packages to link to the exported target or library name.
- For linking proprietary applications, consider changing license to a permissive one; GPLv3 enforces copyleft on derived works.
