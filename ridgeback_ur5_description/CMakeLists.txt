cmake_minimum_required(VERSION 3.10)
project(ridgeback_ur5_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

#install(PROGRAMS
#  scripts/setup_ridgeback_dual_ur10_e_envar
#  scripts/setup_ridgeback_dual_ur10_envar
#  scripts/setup_ridgeback_dual_ur5_e_envar
#  scripts/setup_ridgeback_dual_ur5_envar
#  scripts/setup_ridgeback_ur10_e_envar
#  scripts/setup_ridgeback_ur10_envar
#  scripts/setup_ridgeback_ur5_e_envar
#  scripts/setup_ridgeback_ur5_envar
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()


