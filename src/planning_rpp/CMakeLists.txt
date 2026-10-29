cmake_minimum_required(VERSION 3.8)
project(planning_rpp)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/go_letters scripts/stiction_boost scripts/calibrate_arena
    scripts/turn_rate_controller scripts/calibrate_turning
    scripts/odom_direction_fix scripts/wall_repulsion
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
