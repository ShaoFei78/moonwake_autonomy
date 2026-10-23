cmake_minimum_required(VERSION 3.8)
project(moonwake_ros2_training)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(heartbeat_node src/heartbeat_node.cpp)
ament_target_dependencies(heartbeat_node rclcpp std_msgs)
add_executable(talker_node src/talker_node.cpp)
ament_target_dependencies(talker_node rclcpp std_msgs)
add_executable(listener_node src/listener_node.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)

install(TARGETS heartbeat_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS talker_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS listener_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()