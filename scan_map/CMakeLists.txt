cmake_minimum_required(VERSION 3.5)
project(scan_map)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  templates
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()