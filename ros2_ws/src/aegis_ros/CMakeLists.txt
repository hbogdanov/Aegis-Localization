cmake_minimum_required(VERSION 3.16)
project(aegis_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2 tf2_ros)

install(TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
