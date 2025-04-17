cmake_minimum_required(VERSION 3.10)
project(ros2_mobile_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_subdirectory(ridgeback_ur5_bringup)
add_subdirectory(ridgeback_ur5_description)

ament_package()
