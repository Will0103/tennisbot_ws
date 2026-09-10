cmake_minimum_required(VERSION 3.8)
project(tennisbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cancel_navigation src/cancel_navigation.cpp)
ament_target_dependencies(cancel_navigation rclcpp rclcpp_action nav2_msgs sensor_msgs)

install(TARGETS
  cancel_navigation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
