cmake_minimum_required(VERSION 3.8)
project(moonwake_sensor_status)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sensor_status_node src/sensor_status_node.cpp)
ament_target_dependencies(sensor_status_node rclcpp)

install(TARGETS sensor_status_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
