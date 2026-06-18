cmake_minimum_required(VERSION 3.16)
project(my_arm_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# LibSerial via pkg-config 
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

option(MY_ARM_HARDWARE_ENABLE_INTERNAL_PINOCCHIO
  "Build the hardware plugin with internal C++ Pinocchio dynamics. The normal path uses the Python dynamics_monitor torque topic."
  OFF)
if(MY_ARM_HARDWARE_ENABLE_INTERNAL_PINOCCHIO)
  find_package(pinocchio QUIET)
endif()

## COMPILE
add_library(
  my_arm_hardware
  SHARED
  hardware/myarm.cpp
)
target_compile_features(my_arm_hardware PUBLIC cxx_std_17)
target_include_directories(my_arm_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/my_arm_hardware>
${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(my_arm_hardware PUBLIC
  ${LIBSERIAL_LIBRARIES}
)

if(MY_ARM_HARDWARE_ENABLE_INTERNAL_PINOCCHIO AND pinocchio_FOUND)
  target_compile_definitions(my_arm_hardware PUBLIC MY_ARM_HARDWARE_HAS_PINOCCHIO)
  if(TARGET pinocchio::pinocchio)
    target_link_libraries(my_arm_hardware PUBLIC pinocchio::pinocchio)
  else()
    target_include_directories(my_arm_hardware PUBLIC ${pinocchio_INCLUDE_DIRS})
    target_link_libraries(my_arm_hardware PUBLIC ${pinocchio_LIBRARIES})
  endif()
  message(STATUS "Pinocchio found: enabling dynamics feedforward support in my_arm_hardware")
elseif(MY_ARM_HARDWARE_ENABLE_INTERNAL_PINOCCHIO)
  message(STATUS "Internal Pinocchio dynamics requested but Pinocchio was not found; use dynamics_monitor topic feedforward instead")
else()
  message(STATUS "Internal Pinocchio dynamics disabled; hardware feedforward can use the dynamics_monitor torque topic")
endif()

target_compile_options(my_arm_hardware PRIVATE
  ${LIBSERIAL_CFLAGS_OTHER}
)

ament_target_dependencies(
  my_arm_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface my_arm_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/my_arm_hardware
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/my_arm_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/my_arm_hardware
)
install(
  PROGRAMS
    scripts/homing_button.py
    scripts/dynamics_monitor.py
    scripts/joint_feedback_monitor.py
    scripts/stm32_trace_visualizer.py
    scripts/mit_gain_tuner.py
    scripts/manual_torque_tuner.py
    scripts/arm_emergency_stop.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS my_arm_hardware
  EXPORT export_my_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_my_arm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
