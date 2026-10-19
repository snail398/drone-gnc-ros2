cmake_minimum_required(VERSION 3.16)
project(gnc_ros CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gnc_core REQUIRED)

foreach(node position_monitor mission_node)
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} rclcpp::rclcpp ${px4_msgs_TARGETS} gnc_core::gnc_core)
  target_compile_options(${node} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
