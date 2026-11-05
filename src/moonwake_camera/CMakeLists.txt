cmake_minimum_required(VERSION 3.8)
project(moonwake_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# ⚠️ 2. 查找消息生成器
find_package(rosidl_default_generators REQUIRED)

# 3. 生成自定义消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 4. 添加你的 C++ 节点（failure_recorder.cpp 是异常样本保存模块）
add_executable(buoy_detector_node
  src/buoy_detector_node.cpp
  src/failure_recorder.cpp
)

# 4.1 让 #include "moonwake_camera/failure_recorder.hpp" 能找到；保存模块用到 std::filesystem，需要 C++17
target_include_directories(buoy_detector_node PRIVATE include)
target_compile_features(buoy_detector_node PRIVATE cxx_std_17)

# 5. 链接常规依赖
ament_target_dependencies(buoy_detector_node rclcpp std_msgs cv_bridge sensor_msgs message_filters)

# ⚠️ 6. 【关键步骤】将节点链接到本包生成的消息类型支持上
# 如果不加这一行，你的 C++ 代码将无法 #include 生成的消息头文件，或者链接时报错！
rosidl_target_interfaces(buoy_detector_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 7. 安装节点
install(TARGETS
  buoy_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 7.1 安装 Python 工具脚本，这样可以直接 ros2 run moonwake_camera evaluate_detector.py
install(PROGRAMS
  tools/evaluate_detector.py
  tools/tune_params.py
  tools/video_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 7.2 安装参数文件，方便用 --params-file 加载
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 7.3 安装 launch 文件，这样 ros2 launch moonwake_camera xxx.launch.py 才能找到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()