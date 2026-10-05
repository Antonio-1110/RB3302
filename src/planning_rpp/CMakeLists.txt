cmake_minimum_required(VERSION 3.8)
project(planning_rpp)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/go_letters scripts/stiction_boost
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
