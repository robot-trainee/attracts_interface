cmake_minimum_required(VERSION 3.8)
project(attracts_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(attracts_msgs REQUIRED)
find_package(rabcl REQUIRED)

include_directories(include)

add_executable(gamepad_node
  src/gamepad_node.cpp
)
ament_target_dependencies(gamepad_node
  rclcpp
  sensor_msgs
  attracts_msgs
  rabcl
)

install(TARGETS
  gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
