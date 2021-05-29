cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

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
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# add_executable(name_exec_that_will_be_created path_to_cpp_file)
add_executable(cpp_node src/my_first_node.cpp)
add_executable(robot_news_station src/robot_news_station.cpp)
add_executable(smartphone src/smartphone.cpp)
add_executable(number_publisher src/number_publisher.cpp)
add_executable(number_counter src/number_counter.cpp)
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
add_executable(add_two_ints_client src/add_two_ints_client.cpp)
add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
add_executable(battery src/battery.cpp)
add_executable(led_panel src/led_panel.cpp)

# link dependencies to exec
ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(robot_news_station rclcpp std_msgs)
ament_target_dependencies(smartphone rclcpp std_msgs)
ament_target_dependencies(number_publisher rclcpp std_msgs)
ament_target_dependencies(number_counter rclcpp std_msgs std_srvs)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)
ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)
ament_target_dependencies(battery rclcpp my_robot_interfaces)
ament_target_dependencies(led_panel rclcpp my_robot_interfaces)


# Install exec in the workspace
install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  number_counter
  number_publisher
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hardware_status_publisher
  battery
  led_panel
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
