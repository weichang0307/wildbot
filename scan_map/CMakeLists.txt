cmake_minimum_required(VERSION 3.5)
project(scan_map)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/lidar_mapper.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  scripts
  templates
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()