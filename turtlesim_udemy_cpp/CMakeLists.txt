cmake_minimum_required(VERSION 3.5)
project(turtlesim_udemy_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# add_executables
add_executable(turtle_spawn src/turtle_spawn.cpp)
add_executable(turtle_controller src/turtle_controller.cpp)

#link dependencies
ament_target_dependencies(turtle_spawn rclcpp turtlesim my_robot_interfaces)
ament_target_dependencies(turtle_controller geometry_msgs rclcpp turtlesim my_robot_interfaces)

# Install exec in the workspace
install(TARGETS
  turtle_spawn
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
