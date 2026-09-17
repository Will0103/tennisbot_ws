cmake_minimum_required(VERSION 3.8)
project(tennisbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(patrol_nodes src/patrol_nodes.cpp)
ament_target_dependencies(patrol_nodes rclcpp rclcpp_action nav2_msgs geometry_msgs tf2 tf2_geometry_msgs sensor_msgs std_srvs std_msgs)

install(TARGETS
  patrol_nodes
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY behavior_tree config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
