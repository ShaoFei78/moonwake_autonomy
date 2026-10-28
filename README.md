# Moonwake Autonomy

Moonwake Autonomy 是一个基于 ROS 2 Humble 和 C++ 的入门示例工作区。项目包含两个 ROS 2 Package，用于练习节点、Topic、Service、参数、Launch 文件以及简单的发布/订阅数据流。

当前代码不连接真实相机或其他传感器，也不包含车辆控制算法。相机状态、目标偏移和转向提示均为模拟数据。

## 当前功能

### `moonwake_ros2_training`

| 可执行文件 | Node 名称 | 主要接口 | 说明 |
| --- | --- | --- | --- |
| `heartbeat_node` | `heartbeat_node` | 无 | 每秒输出一次 `node is alive` |
| `talker_node` | `talker_node` | 发布 `/chatter` | 每 500 ms 发布一条 `std_msgs/msg/String` |
| `listener_node` | `listener_node` | 订阅 `/chatter` | 接收并保存最新消息，默认不打印消息 |
| `target_offset_publisher` | `target_offset_publisher` | 发布 `target_offset` | 按顺序发布模拟横向目标偏移，类型为 `std_msgs/msg/Float64` |
| `steering_hint_node` | `steering_hint_node` | 订阅 `target_offset`，发布 `steering_hint` | 根据偏移和死区生成 `LEFT`、`RIGHT` 或 `STRAIGHT` |

该 Package 还提供 `launch/training.launch.py` 和 `config/training_params.yaml`，用于同时启动目标偏移发布节点与转向提示节点。

### `moonwake_sensor_status`

| 可执行文件 | Node 名称 | 主要接口 | 说明 |
| --- | --- | --- | --- |
| `sensor_status_node` | `front_camera_status` | Service `set_detailed_log` | 每秒输出模拟相机状态；可通过 `std_srvs/srv/SetBool` 切换详细日志 |

## 数据流

使用 Launch 文件运行时：

```text
target_offset_publisher
        │
        │ /training/target_offset  (std_msgs/msg/Float64)
        ▼
steering_hint_node
        │
        │ /training/steering_hint  (std_msgs/msg/String)
        ▼
其他订阅者或 ros2 topic echo
```

基础发布/订阅示例：

```text
talker_node ── /chatter (std_msgs/msg/String) ──► listener_node
```

`target_offset_publisher` 在 Launch 文件中发布的消息依次为 `-0.8`、`-0.4`、`0.0`、`0.4`、`0.8`，随后循环。`steering_hint_node` 根据 `deadband` 参数判断方向：偏移绝对值不超过死区时为 `STRAIGHT`，小于负死区时为 `LEFT`，大于正死区时为 `RIGHT`。

## 项目结构

```text
moonwake_autonomy-main/
├── README.md
├── .gitignore
└── src/
    ├── moonwake_ros2_training/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── training_params.yaml
    │   ├── launch/
    │   │   └── training.launch.py
    │   └── src/
    │       ├── heartbeat_node.cpp
    │       ├── listener_node.cpp
    │       ├── steering_hint_node.cpp
    │       ├── talker_node.cpp
    │       └── target_offset_publisher.cpp
    └── moonwake_sensor_status/
        ├── CMakeLists.txt
        ├── LICENSE
        ├── package.xml
        ├── README.md
        └── src/
            └── sensor_status_node.cpp
```

注意：`moonwake_sensor_status/CMakeLists.txt` 当前仍会安装 `launch/` 和 `config/` 目录，但 Package 中尚未提交这两个目录。首次构建前需要按后文命令创建空目录，否则安装阶段可能报目录不存在。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8 或更高版本
- C++ 编译器
- `colcon`
- `ament_cmake`
- `rclcpp`
- `std_msgs`
- `std_srvs`
- `launch_ros` / `ros2launch`

如果使用 Windows，建议在 WSL 2 的 Ubuntu 22.04 中运行。以下命令均以 Linux Shell 为例。

## 安装基础依赖

先加载 ROS 2 环境并确认命令可用：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

安装 `colcon`、编译依赖和 Launch 运行依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  ros-humble-ros2launch
```

如果尚未初始化 `rosdep`，可以执行：

```bash
sudo rosdep init
rosdep update
```

如果 `rosdep init` 提示配置文件已存在，可以跳过该命令。

## 创建工作区

将两个 Package 放入工作区的 `src/` 目录。下面的示例使用 `~/ros2_ws`：

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/moonwake_autonomy-main/src/moonwake_ros2_training ~/ros2_ws/src/
cp -r /path/to/moonwake_autonomy-main/src/moonwake_sensor_status ~/ros2_ws/src/
cd ~/ros2_ws
```

请将 `/path/to/moonwake_autonomy-main` 替换为项目的实际路径。

安装工作区中声明的 ROS 依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

创建 `moonwake_sensor_status` 的 CMake 安装目录：

```bash
mkdir -p \
  ~/ros2_ws/src/moonwake_sensor_status/launch \
  ~/ros2_ws/src/moonwake_sensor_status/config
```

这两个目录当前可以为空。

## 构建

### 一次构建两个 Package

在工作区根目录执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select moonwake_ros2_training moonwake_sensor_status
source install/setup.bash
```

### 只构建训练示例

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select moonwake_ros2_training
source install/setup.bash
```

### 只构建相机状态示例

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select moonwake_sensor_status
source install/setup.bash
```

每次打开新终端，都应加载 ROS 2 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

确认可执行文件已安装：

```bash
ros2 pkg executables moonwake_ros2_training
ros2 pkg executables moonwake_sensor_status
```

预期可以分别看到 5 个训练节点和 1 个传感器状态节点。

## 运行训练 Launch

在工作区环境中运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch moonwake_ros2_training training.launch.py
```

该命令会同时启动：

- `target_offset_publisher`
- `steering_hint_node`

并应用以下 Topic 重映射：

- `target_offset` → `/training/target_offset`
- `steering_hint` → `/training/steering_hint`

Launch 默认读取 `config/training_params.yaml`：

```yaml
target_offset_publisher:
  ros__parameters:
    amplitude: 0.8
    publish_hz: 1.0

steering_hint_node:
  ros__parameters:
    deadband: 0.10
```

参数含义：

| 参数 | 所属 Node | Launch 默认值 | 说明 |
| --- | --- | --- | --- |
| `amplitude` | `target_offset_publisher` | `0.8` | 目标偏移序列的幅值 |
| `publish_hz` | `target_offset_publisher` | `1.0` | 发布频率，必须大于 0 |
| `deadband` | `steering_hint_node` | `0.10` | 方向判断死区 |

上表是 `training_params.yaml` 中的值。直接使用 `ros2 run` 且不传入参数时，源码内部默认值为 `amplitude=0.5`、`publish_hz=1.0`、`deadband=0.05`。

在另一个终端查看偏移和转向提示：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /training/target_offset --once
```

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /training/steering_hint --once
```

修改 YAML 参数后，重新构建并重新运行 Launch：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select moonwake_ros2_training
source install/setup.bash
ros2 launch moonwake_ros2_training training.launch.py
```

按 `Ctrl+C` 可以停止 Launch 及其启动的节点。

## 单独运行节点

以下命令可以分别粘贴到不同终端中执行。运行前都要加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 心跳节点

```bash
ros2 run moonwake_ros2_training heartbeat_node
```

预期每秒输出：

```text
[INFO] [...] [heartbeat_node]: node is alive
```

### 消息发布节点

```bash
ros2 run moonwake_ros2_training talker_node
```

该节点每 500 ms 向 `/chatter` 发布：

```text
Hello,ShaoFei.
```

### 消息订阅节点

```bash
ros2 run moonwake_ros2_training listener_node
```

`listener_node` 会订阅 `/chatter` 并保存最新消息，但当前实现不会把消息打印到终端。请先运行 `talker_node`，再使用下面的命令检查消息：

```bash
ros2 topic echo /chatter --once
```

### 目标偏移发布节点

使用源码默认参数单独运行：

```bash
ros2 run moonwake_ros2_training target_offset_publisher
```

覆盖默认参数：

```bash
ros2 run moonwake_ros2_training target_offset_publisher --ros-args \
  -p amplitude:=0.8 \
  -p publish_hz:=1.0
```

单独运行时不使用 Launch 的 Topic 重映射，默认 Topic 为 `/target_offset`。

### 转向提示节点

使用源码默认参数单独运行：

```bash
ros2 run moonwake_ros2_training steering_hint_node
```

覆盖死区参数：

```bash
ros2 run moonwake_ros2_training steering_hint_node --ros-args \
  -p deadband:=0.10
```

单独运行时使用默认 Topic `/target_offset` 和 `/steering_hint`。请先启动 `target_offset_publisher`，再启动该节点。

## 运行相机状态节点

在已加载工作区环境的终端中执行：

```bash
ros2 run moonwake_sensor_status sensor_status_node
```

默认每秒输出：

```text
[INFO] [...] [front_camera_status]: front camera status: READY
```

打开详细日志：

```bash
ros2 service call /set_detailed_log std_srvs/srv/SetBool "{data: true}"
```

开启后，每秒输出包含模拟 FPS、温度和延迟：

```text
[DEBUG] front camera status: READY | FPS: 30 , Temp: 45C , Latency:12ms
```

关闭详细日志：

```bash
ros2 service call /set_detailed_log std_srvs/srv/SetBool "{data: false}"
```

该节点只模拟状态，不连接真实相机，也不发布图像。

## 检查运行状态

查看当前运行的所有节点：

```bash
ros2 node list
```

运行训练 Launch 后应包含：

```text
/target_offset_publisher
/steering_hint_node
```

运行其他节点后，Node 列表还会出现：

```text
/heartbeat_node
/listener_node
/talker_node
/front_camera_status
```

查看训练 Topic：

```bash
ros2 topic list
ros2 topic info /training/target_offset
ros2 topic info /training/steering_hint
ros2 topic hz /training/target_offset
```

使用默认配置时，`/training/target_offset` 的发布频率应接近 1 Hz。

查看 `front_camera_status` 提供的 Service：

```bash
ros2 service list
ros2 service type /set_detailed_log
ros2 interface show std_srvs/srv/SetBool
```

查看 Node 参数：

```bash
ros2 param list /target_offset_publisher
ros2 param list /steering_hint_node
```

`target_offset_publisher` 应提供 `amplitude` 和 `publish_hz`；`steering_hint_node` 应提供 `deadband`。

## 常见问题

### 找不到 `ros2`

当前终端没有加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
```

### 找不到 Package 或可执行文件

先确认工作区已构建，然后重新加载环境：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select moonwake_ros2_training moonwake_sensor_status
source install/setup.bash
ros2 pkg executables moonwake_ros2_training
ros2 pkg executables moonwake_sensor_status
```

### 构建 `moonwake_sensor_status` 时提示找不到 `launch` 或 `config`

该 Package 的 CMake 会安装这两个目录，但仓库尚未包含空目录。创建后重新构建：

```bash
mkdir -p \
  ~/ros2_ws/src/moonwake_sensor_status/launch \
  ~/ros2_ws/src/moonwake_sensor_status/config
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select moonwake_sensor_status
source install/setup.bash
```

### `listener_node` 启动后没有日志

这是当前实现的预期行为。节点会接收消息，但不会打印。使用 `ros2 topic echo /chatter --once` 检查 `talker_node` 是否正在发布。

### 看不到训练 Topic

确认当前使用哪种启动方式：

- `ros2 launch` 使用 `/training/target_offset` 和 `/training/steering_hint`；
- 单独运行节点使用 `/target_offset` 和 `/steering_hint`。

查看所有 Topic：

```bash
ros2 topic list
```

### Service 调用失败

确认 `sensor_status_node` 正在运行，并检查 Service 名称：

```bash
ros2 node list
ros2 service list
```

默认 Service 为 `/set_detailed_log`。

## 当前限制

- 所有状态和消息均为模拟数据，不连接真实硬件；
- `listener_node` 只保存最新消息，没有终端输出；
- Sensor Status Package 目前没有自己的 Launch 文件；
- Sensor Status Package 的 CMake 引用了尚未提交的 `launch/`、`config/` 目录，首次构建前需要手动创建。
