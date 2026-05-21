cmake_minimum_required(VERSION 3.5)
project(localization)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/lidar_localizer.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
