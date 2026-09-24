cmake_minimum_required(VERSION 3.8)
project(tennisbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# LibSerial
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED IMPORTED_TARGET libserial)

# =========================
# Hardware Interface Plugin
# =========================

add_library(${PROJECT_NAME} SHARED
  src/tennisbot_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}
  PkgConfig::SERIAL
)

pluginlib_export_plugin_description_file(
  hardware_interface
  tennisbot_interface.xml
)

# =========================
# Install C++ Plugin
# =========================

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Python
# =========================

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  ${PROJECT_NAME}/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Export
# =========================

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()