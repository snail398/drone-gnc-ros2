cmake_minimum_required(VERSION 3.16)
project(gnc_core CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(gnc_core
  src/control/pid.cpp
)
target_include_directories(gnc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gnc_core PUBLIC Eigen3::Eigen)
target_compile_options(gnc_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gnc_core EXPORT export_gnc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_gnc_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid gnc_core)
endif()

ament_package()
