cmake_minimum_required(VERSION 3.5)
project(my_robot_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# this will build the message: name of the package (=name of the project) + file.
# After the CMake and the package have been modified to generate the interfaces,
# it is just necessary to add the files down here.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/ComputeRectangleArea.srv"
  "msg/LedStates.msg"
  "srv/SetLed.srv"
  "msg/AvailableTurtles.msg"
)

ament_package()
