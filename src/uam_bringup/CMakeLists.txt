cmake_minimum_required(VERSION 3.16)
project(uam_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch doc
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/state_machine_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
