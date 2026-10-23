# moonwake_sensor_status

一个用于 ROS 2 Humble 入门的 C++ Package。它启动名为 `front_camera_status` 的 Node，每秒记录一次模拟的前向相机状态。程序不连接真实相机，也不输出图像或控制指令。

## 目录

- `src/sensor_status_node.cpp`：Node 与周期性状态日志；
- `package.xml`：Package 元数据和依赖；
- `CMakeLists.txt`：构建与安装配置。

## 依赖

- Ubuntu 22.04；
- ROS 2 Humble；
- `ament_cmake`、`rclcpp`。

## 构建与运行

将本 Package 放入 ROS 2 工作区的 `src/`，在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select moonwake_sensor_status
source install/setup.bash
ros2 run moonwake_sensor_status sensor_status_node
```

在另一个已加载相同工作区的终端中，可使用 `ros2 node list` 和 `ros2 node info /front_camera_status` 观察运行状态。按 `Ctrl+C` 退出。
