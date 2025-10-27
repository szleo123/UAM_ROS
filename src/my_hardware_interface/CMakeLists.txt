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
