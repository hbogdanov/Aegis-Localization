cmake_minimum_required(VERSION 3.16)
project(aegis_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aegis_core REQUIRED)
find_package(aegis_msgs REQUIRED)

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2 tf2_ros aegis_core aegis_msgs Eigen3)
target_link_libraries(ekf_node ${aegis_core_LIBRARIES} Eigen3::Eigen)

add_executable(trajectory_logger_node src/trajectory_logger_node.cpp)
ament_target_dependencies(trajectory_logger_node rclcpp nav_msgs geometry_msgs ament_index_cpp Eigen3)

target_link_libraries(trajectory_logger_node Eigen3::Eigen)

add_executable(fake_sensor_publisher_node src/fake_sensor_publisher_node.cpp)
ament_target_dependencies(fake_sensor_publisher_node rclcpp nav_msgs sensor_msgs geometry_msgs)

install(TARGETS ekf_node trajectory_logger_node fake_sensor_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
