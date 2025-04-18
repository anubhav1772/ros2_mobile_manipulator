cmake_minimum_required(VERSION 3.10)
project(ridgeback_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

#ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

