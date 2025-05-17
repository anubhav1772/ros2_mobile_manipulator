cmake_minimum_required(VERSION 3.10)
project(ridgeback_ur5_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch scripts utils
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/arm_reset.py
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(utils)
ament_python_install_module(utils/launch_common.py)

ament_package()

