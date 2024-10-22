cmake_minimum_required(VERSION 3.8)
project(attracts_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(gamepad_node src/gamepad_node.cpp)
ament_target_dependencies(gamepad_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
