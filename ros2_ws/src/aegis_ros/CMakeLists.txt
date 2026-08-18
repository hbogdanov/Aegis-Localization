cmake_minimum_required(VERSION 3.16)
project(aegis_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
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

add_executable(ukf_node src/ukf_node.cpp)
ament_target_dependencies(ukf_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2 tf2_ros aegis_core Eigen3)
target_link_libraries(ukf_node ${aegis_core_LIBRARIES} Eigen3::Eigen)

add_executable(particle_filter_node src/particle_filter_node.cpp)
ament_target_dependencies(particle_filter_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2 tf2_ros aegis_core aegis_msgs Eigen3)
target_link_libraries(particle_filter_node ${aegis_core_LIBRARIES} Eigen3::Eigen)

add_executable(gazebo_ground_truth_bridge_node src/gazebo_ground_truth_bridge_node.cpp)
ament_target_dependencies(gazebo_ground_truth_bridge_node rclcpp nav_msgs geometry_msgs)

add_executable(circle_command_publisher_node src/circle_command_publisher_node.cpp)
ament_target_dependencies(circle_command_publisher_node rclcpp geometry_msgs)

add_executable(trajectory_logger_node src/trajectory_logger_node.cpp)
ament_target_dependencies(trajectory_logger_node rclcpp nav_msgs geometry_msgs ament_index_cpp Eigen3)

target_link_libraries(trajectory_logger_node Eigen3::Eigen)

add_executable(fake_sensor_publisher_node src/fake_sensor_publisher_node.cpp)
ament_target_dependencies(fake_sensor_publisher_node rclcpp nav_msgs sensor_msgs geometry_msgs)

add_executable(euroc_replay_node src/euroc_replay_node.cpp)
ament_target_dependencies(euroc_replay_node rclcpp nav_msgs sensor_msgs geometry_msgs)

install(TARGETS ekf_node ukf_node particle_filter_node gazebo_ground_truth_bridge_node circle_command_publisher_node trajectory_logger_node fake_sensor_publisher_node euroc_replay_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME})

ament_package()
