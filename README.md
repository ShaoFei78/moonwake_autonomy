# Moonwake Autonomy

Moonwake Autonomy 是一个基于 ROS 2 与 C++ 的入门示例工作区，包含三个训练用节点和一个模拟相机状态节点。项目用于练习 ROS 2 Package 的构建、Node 启动、Topic 发布与订阅，以及周期性状态日志。

当前项目不连接真实相机、雷达或其他硬件，也不包含自动驾驶控制算法。所有传感器状态和消息均为模拟数据。

## 功能概览

| Package | 可执行文件 | Node 名称 | 作用 |
| --- | --- | --- | --- |
| `moonwake_ros2_training` | `heartbeat_node` | `heartbeat_node` | 每秒输出一次 `node is alive`，用于确认节点正常存活 |
| `moonwake_ros2_training` | `talker_node` | `talker_node` | 每 500 ms 向 `/chatter` 发布 `Hello,ShaoFei.` |
| `moonwake_ros2_training` | `listener_node` | `listener_node` | 订阅 `/chatter` 并保存最近一条消息；默认不打印消息,请使用ros2 topic echo /topic --once查看消息|
| `moonwake_sensor_status` | `sensor_status_node` | `front_camera_status` | 每秒输出一次模拟前向相机的 `READY` 状态 |

## 项目结构

```text
moonwake_autonomy-main/
├── README.md
├── .gitignore
└── src/
    ├── moonwake_ros2_training/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       ├── heartbeat_node.cpp
    │       ├── listener_node.cpp
    │       └── talker_node.cpp
    └── moonwake_sensor_status/
        ├── CMakeLists.txt
        ├── LICENSE
        ├── package.xml
        ├── README.md
        └── src/
            └── sensor_status_node.cpp
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8 或更高版本
- C++ 编译器
- `colcon`
- `ament_cmake`、`rclcpp`、`std_msgs`


构建命令
```bash
source /opt/ros/humble/setup.bash
//选择编译任务2的packages
colcon build --symlink-install --packages-select moonwake_ros2_training
//选择编译任务1的packages
colcon build --symlink-install --packages-select moonwake_sensor_status
source install/setup.bash
```
