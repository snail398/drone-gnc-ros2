cmake_minimum_required(VERSION 3.16)
project(gnc_ros CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gnc_core REQUIRED)

add_executable(position_monitor src/position_monitor.cpp)
target_link_libraries(position_monitor rclcpp::rclcpp ${px4_msgs_TARGETS} gnc_core::gnc_core)
target_compile_options(position_monitor PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS position_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()
