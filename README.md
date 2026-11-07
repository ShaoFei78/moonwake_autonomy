# Moonwake Autonomy

基于 **ROS 2 Humble + OpenCV** 的视觉检测工作区，目前只包含一个包 `moonwake_camera`：用 HSV 阈值 + 连通域分析从相机图像中检测浮标，并附带失败样本记录、参数调试和性能评估工具。

> 尚未接入真实相机，可用视频文件代替，见 [测试脚本](#测试脚本)。

---

## 运行环境

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（WSL 2 亦可） |
| ROS 2 | Humble |
| C++ 标准 | C++17（用到 `std::filesystem`） |
| OpenCV | 4.x（Ubuntu 22.04 自带 4.5.4，读视频需 FFMPEG 支持） |
| Python | 3.10，依赖 `rclpy`、`cv2`、`numpy` |
| 构建工具 | colcon + ament_cmake |

```bash
sudo apt install ros-humble-cv-bridge ros-humble-message-filters \
                 python3-opencv python3-numpy
```

---

## 架构与数据流

```text
                     ┌────────────────────────────┐
   真实相机 / 视频 ──►│ image_raw     sensor_msgs/Image
                     │ camera_info   sensor_msgs/CameraInfo
                     └──────────────┬─────────────┘
                                    │ ApproximateTime 时间同步
                                    ▼
                     ┌──────────────────────────────────┐
                     │        buoy_detector_node        │
                     │  ① 分辨率一致性检查               │
                     │  ② 数据年龄检查  → INPUT_STALE    │
                     │  ③ 参数合法性检查 → CONFIG_ERROR  │
                     │  ④ BGR→HSV → inRange             │
                     │  ⑤ 形态学开运算 / 闭运算          │
                     │  ⑥ 连通域 + 面积过滤              │
                     └──┬────────────┬────────────┬─────┘
                        │            │            │
          detection ────┘   debug_mask│            │ failure_samples/
        (DetectionArray)      (Image) │            │ 原图 + mask + txt
                        ▼            ▼            ▼
              evaluate_detector   rqt_image_view  tune_params.py
                 → eval_result.csv                （滑动条调参）
```

---

## 包结构

```text
moonwake_autonomy/
└── src/moonwake_camera/
    ├── CMakeLists.txt / package.xml
    ├── msg/                          # Detection.msg / DetectionArray.msg
    ├── include/moonwake_camera/      # failure_recorder.hpp
    ├── src/                          # buoy_detector_node.cpp / failure_recorder.cpp
    ├── config/detector_params.yaml   # 推荐参数
    ├── launch/test_with_video.launch.py
    ├── tools/
    │   ├── video_publisher.py        # 视频代替相机
    │   ├── evaluate_detector.py      # 性能评估
    │   └── tune_params.py            # 滑动条调参
    └── failure_samples/              # 运行时生成（已 gitignore）
```

---

## 节点与可执行文件

| 可执行文件 | 类型 | 说明 |
| --- | --- | --- |
| `buoy_detector_node` | C++ 节点 | 主检测节点，ROS 名 `buoy_detector_node` |
| `video_publisher.py` | Python 节点 | 读视频文件，充当相机发布图像 |
| `evaluate_detector.py` | Python 节点 | 订阅检测结果，统计指标并写 CSV |
| `tune_params.py` | Python 工具 | 带滑动条的离线调参（不发布话题） |

```bash
ros2 pkg executables moonwake_camera
```

---

## 话题接口

### 订阅

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `image_raw` | `sensor_msgs/msg/Image` | 原始图像，要求编码 `bgr8` |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | 与图像做时间同步，并校验分辨率 |

### 发布

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `detection` | `moonwake_camera/msg/DetectionArray` | 每帧的状态与检测结果 |
| `debug_mask` | `sensor_msgs/msg/Image` | 二值 mask（`mono8`），需 `publish_debug_mask:=true` |
| `debug` | `sensor_msgs/msg/Image` | 检测框标注图（`bgr8`），需 `publish_debug_image:=true`（默认开） |

> ⚠️ 检测结果话题是 **`detection`（单数）**，而评估脚本默认订阅 `detections`，需要显式指定。

### `DetectionArray` 字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 继承输入图像的时间戳与坐标系 |
| `status` | `uint8` | 本帧状态码，见下表 |
| `detections` | `Detection[]` | 目标列表 |
| `process_time_ms` | `float32` | 本帧算法耗时 |
| `data_age_ms` | `float32` | 数据年龄（now − 图像时间戳） |

| 值 | 常量 | 含义 |
| --- | --- | --- |
| 0 | `STATUS_DETECTED` | 正常检测到目标 |
| 1 | `STATUS_NO_DETECTION` | 流程正常但无目标 |
| 2 | `STATUS_INPUT_ERROR` | 分辨率不匹配 / 图像解码失败 |
| 3 | `STATUS_INPUT_STALE` | 数据年龄超过 `stale_threshold_sec` |
| 4 | `STATUS_CONFIG_ERROR` | 参数非法（如 `h_min >= h_max`） |

`Detection` 字段：`class_name`、`center`（像素坐标）、`bbox`（`[x_min, y_min, x_max, y_max]`）、`score`（连通域面积 / 外接框面积）。

---

## 参数

### 检测参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `h_min` / `h_max` | `0` / `180` | HSV 色相范围（OpenCV 中 H 为 0~180） |
| `s_min` / `s_max` | `50` / `255` | 饱和度范围 |
| `v_min` / `v_max` | `50` / `255` | 明度范围 |
| `min_area` | `100` | 连通域最小面积（像素），小于则丢弃 |
| `kernel_size` | `5` | 形态学核边长，`Size(k, k)` 椭圆核 |
| `stale_threshold_sec` | `0.5` | 数据年龄阈值（秒），超过判为 `INPUT_STALE` |
| `publish_debug_mask` | `false` | 是否发布 `debug_mask`（二值 mask） |
| `publish_debug_image` | `true` | 是否发布 `debug`（检测框标注图） |

### 异常样本保存参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `save_failure_samples` | `true` | 是否保存（仅 `INPUT_STALE` / `NO_DETECTION` / `CONFIG_ERROR` 三类） |
| `failure_dir` | `""` | 保存目录；**留空 = 自动存到 `<本包>/failure_samples/`** |
| `failure_min_interval_sec` | `10.0` | 两次落盘的最小间隔，防止写满磁盘 |
| `failure_max_samples` | `200` | 单次运行最多保存份数，`<=0` 不限 |

运行时可热改：

```bash
ros2 param set /buoy_detector_node min_area 60
ros2 param get /buoy_detector_node h_min
```

---

## 构建

```bash
cd ~/moonwake_autonomy
source /opt/ros/humble/setup.bash
colcon build --packages-select moonwake_camera \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 会生成 `build/moonwake_camera/compile_commands.json`，VSCode IntelliSense 靠它自动对齐编译参数。**改完代码要重新构建一次**才会刷新。

---

## 运行

### 一条命令跑完整链路（推荐）

```bash
ros2 launch moonwake_camera test_with_video.launch.py \
  video:=/home/shaofei/Videos/test.mp4 rate:=10.0
```

同时启动「视频发布 + 检测 + 评估」，`Ctrl+C` 后自动打印统计报告并保存 CSV。

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video` | `''` | 视频路径；留空则跳过发布节点（接真相机时用） |
| `rate` | `0.0` | 发布帧率，0 = 跟随视频自身 FPS（推荐）；想放慢可填更小值 |
| `loop` | `true` | 播完是否循环 |
| `params_file` | 包内 `config/detector_params.yaml` | 参数文件 |
| `start_evaluator` | `true` | 是否启动评估节点 |
| `eval_topic` | `/detection` | 评估订阅的话题 |
| `eval_csv` | `eval_result.csv` | CSV 输出路径（相对执行目录） |
| `print_interval` | `5.0` | 进度打印间隔（秒），0 关闭 |

### 手动分终端启动

```bash
# 终端 1：图像源
ros2 run moonwake_camera video_publisher.py --ros-args -p video:=/path/test.mp4 -p rate:=10.0

# 终端 2：检测节点
ros2 run moonwake_camera buoy_detector_node --ros-args \
  --params-file ~/moonwake_autonomy/src/moonwake_camera/config/detector_params.yaml
```

### 观察结果

```bash
ros2 topic echo /detection --once
ros2 run rqt_image_view rqt_image_view /image_raw     # 原始画面
ros2 run rqt_image_view rqt_image_view /debug         # 检测框标注图（最常用）
ros2 run rqt_image_view rqt_image_view /debug_mask    # 二值 mask
```

---

## 测试脚本

### `video_publisher.py` —— 用视频代替相机

按固定帧率发布 `image_raw` + `camera_info`，让检测节点像接真相机一样工作。

```bash
ros2 run moonwake_camera video_publisher.py --ros-args \
  -p video:=/home/shaofei/videos/test.mp4 -p rate:=10.0
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video` | `''` | 视频路径（必填） |
| `rate` | `0.0` | 发布帧率，0 = 跟随视频 FPS |
| `loop` | `true` | 播完循环 |
| `encoding` | `bgr8` | 发布图像编码，可选 `bgr8` / `rgb8` / `mono8` / `bgra8` |
| `frame_id` | `camera` | `header.frame_id` |
| `qos_depth` | `10` | 发布队列深度 |

> 时间戳用**当前 ROS 时间**，不是视频原始时间；否则 `data_age` 很大，每帧都会被判成 `INPUT_STALE`。
>
> `encoding` 也可以在参数文件的 `video_publisher:` 段里配置（launch 会同时把参数文件传给发布节点）。
> ⚠️ 改成 `mono8` 后图像没有颜色，HSV 阈值检测必然一帧都检不出——它只是用来验证链路，不适合做彩色目标检测。

### `evaluate_detector.py` —— 性能评估

订阅检测结果，统计帧数、状态分布、耗时、延迟，逐帧写 CSV，退出时打印报告。

```bash
ros2 run moonwake_camera evaluate_detector.py --ros-args \
  -p topic:=/detection -p output_csv:=eval_result.csv
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `topic` | `detections` | 订阅话题（⚠️ 检测节点是 `/detection`，需显式指定） |
| `output_csv` | `eval_result.csv` | 输出路径，相对执行目录 |
| `print_interval` | `5.0` | 进度打印间隔，`<=0` 关闭 |
| `qos_depth` | `10` | 订阅队列深度 |

CSV 表头：`Timestamp, Status, Process_Time_ms, Data_Age_ms, Num_Detections`。
`Timestamp` 取图像 `header.stamp`（秒）；耗时只统计 `process_time_ms > 0` 的帧。

### `tune_params.py` —— 滑动条调参

离线读取一张图片，用滑动条实时调整 HSV 阈值、`min_area`、`kernel_size`，处理流程与 C++ 节点完全一致（已验证逐像素相同）。

```bash
# 指定图片
python3 ~/moonwake_autonomy/src/moonwake_camera/tools/tune_params.py 图片.png

# 不带参数：自动挑 failure_samples 里最新的原图
python3 ~/moonwake_autonomy/src/moonwake_camera/tools/tune_params.py

# 无界面批处理，导出 mask
python3 .../tune_params.py 图片.png --dump out_mask.png -p h_min=34 -p min_area=60
```

快捷键：`p` 打印可直接复制的 `ros2 run` 参数，`s` 保存 mask 与预览图，`q` / `ESC` 退出。

---

## 配置文件

`config/detector_params.yaml` 一个文件配置两个节点，各读各的段落：

```yaml
buoy_detector_node:
  ros__parameters:
    h_min: 35
    ...
    save_failure_samples: true      # 异常样本总开关
    failure_min_interval_sec: 10.0  # 两次保存的最小间隔（秒）

video_publisher:
  ros__parameters:
    encoding: bgr8                  # bgr8 / rgb8 / mono8 / bgra8
```

```bash
# 单独运行检测节点
ros2 run moonwake_camera buoy_detector_node --ros-args \
  --params-file ~/moonwake_autonomy/src/moonwake_camera/config/detector_params.yaml

# launch 会把这个文件同时传给检测节点和视频发布节点
ros2 launch moonwake_camera test_with_video.launch.py video:=/path/test.mp4
```

> YAML 顶层 key 必须与节点名一致（`buoy_detector_node` / `video_publisher`），否则参数静默不生效。

---

## 失败样本

出现 `INPUT_STALE` / `NO_DETECTION` / `CONFIG_ERROR` 时自动保存到 `failure_samples/`，每份三个文件：

```text
fail_20261105_170656_758_0001_no_detection.png        # 原图
fail_20261105_170656_758_0001_no_detection_mask.png   # mask
fail_20261105_170656_758_0001_no_detection.txt        # 参数 + 状态码 + 诊断信息
```

文件名 = `fail_<时间戳>_<序号>_<状态>`，带序号且不覆盖，默认每 10 秒最多存 1 份。

---

## 已知限制

- 尚未接入真实相机驱动。用 `ros2 bag play` 回放时需给检测节点加 `use_sim_time:=true`，否则会被判为数据过期。
- 检测基于 HSV 阈值 + 连通域，不是深度学习；单区间 `inRange` 对红色这类跨越 H 边界（0 与 180 两侧）的物体会漏检一半。
- `video_publisher.py` 用 `array.array('B')` 填充 `Image.data`；自己写发布端时若直接赋 `bytes`，每帧会慢约 100 倍（2400×1080 下 640 ms vs 5 ms），进而导致全部帧被判为 `INPUT_STALE`。
- `min_area` 是像素面积，依赖分辨率，换分辨率后参数需重新调。
