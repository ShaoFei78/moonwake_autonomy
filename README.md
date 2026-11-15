# Moonwake Autonomy

基于 **ROS 2 Humble + OpenCV** 的视觉检测工作区，目前只包含一个包 `moonwake_camera`：
用 **HSV 阈值**或 **YOLO11（ONNX）** 从相机图像中检测浮标，并用**卡尔曼滤波 + 数据关联**给出跨帧稳定的目标编号，
附带失败样本记录、参数调试和性能评估工具。

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
                     │  ③ detector_->detect(bgr)        │
                     └──────────────┬───────────────────┘
                                    │ IDetector 抽象接口
                     ┌──────────────┴───────────────┐
                     ▼                              ▼
            ┌─────────────────┐            ┌─────────────────┐
            │  HsvDetector    │            │  YoloDetector   │
            │ OpenCV 颜色阈值  │            │ ONNX Runtime    │
            │ +自适应+连通域   │            │ + letterbox/NMS │
            └─────────────────┘            └─────────────────┘
                     │ 两个后端都返回同样结构的 Detection（原图像素坐标）
                     ▼
     ┌───────────────┼───────────────┬──────────────────┐
     ▼               ▼               ▼                  ▼
 detection        debug         debug_mask        failure_samples/
(DetectionArray) (标注图)      (仅 hsv 后端)      原图 + mask + txt
     │
     ▼
 evaluate_detector → eval_result.csv
```

---

> 检测之后还有一步**目标跟踪**：给每个框分配跨帧稳定的 `track_id`，并给出
> `age_s` / `valid_until` / `quality`。短时遮挡时会通过 `predicted_tracks` 发布预测。
> 详见 [目标跟踪](#目标跟踪)。

## 包结构

```text
moonwake_autonomy/
└── src/moonwake_camera/
    ├── CMakeLists.txt / package.xml
    ├── msg/                          # Detection.msg / DetectionArray.msg
    ├── include/moonwake_camera/      # detector.hpp / hsv_detector.hpp / yolo_detector.hpp / failure_recorder.hpp
    ├── src/                          # 节点 + 两个检测后端 + 工厂 + 失败样本记录
    ├── config/detector_params.yaml   # 推荐参数
    ├── launch/test_with_video.launch.py  # 视频 + 检测 + 评估 一键启动
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
| `debug_mask` | `sensor_msgs/msg/Image` | 二值 mask（`mono8`），需 `publish_debug_mask:=true`；**仅 `hsv` 后端有** |
| `debug` | `sensor_msgs/msg/Image` | 检测框标注图（`bgr8`），需 `publish_debug_image:=true`（默认开）；右上角叠加实时 FPS |

> ⚠️ 检测结果话题是 **`detection`（单数）**，而评估脚本默认订阅 `detections`，需要显式指定。

### `DetectionArray` 字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 继承输入图像的时间戳与坐标系 |
| `status` | `uint8` | 本帧状态码，见下表 |
| `detections` | `Detection[]` | **本帧真实观测到的**目标（可能为空） |
| `predicted_tracks` | `Detection[]` | 本帧**只有预测、没有观测**的目标（`track_state = TRACK_LOST`），可能为空 |
| `process_time_ms` | `float32` | 本帧算法耗时（含检测 + 跟踪） |
| `data_age_ms` | `float32` | 数据年龄（now − 图像时间戳） |

| 值 | 常量 | 含义 |
| --- | --- | --- |
| 0 | `STATUS_DETECTED` | 正常检测到目标 |
| 1 | `STATUS_NO_DETECTION` | 流程正常但无目标 |
| 2 | `STATUS_INPUT_ERROR` | 分辨率不匹配 / 图像解码失败 |
| 3 | `STATUS_INPUT_STALE` | 数据年龄超过 `stale_threshold_sec` |
| 4 | `STATUS_CONFIG_ERROR` | 参数非法（如 `h_min >= h_max`、`model_path` 为空） |
| 5 | `STATUS_MODEL_ERROR` | 模型加载失败 / 推理抛异常（不会伪装成 `NO_DETECTION`） |

`Detection` 字段：`class_name`、`center`（像素坐标）、`bbox`（`[x_min, y_min, x_max, y_max]`）、`score`
（语义由后端定义，跨后端不可直接比较：hsv = 连通域填充率，yolo = 模型置信度），
以及跟踪字段 `track_id` / `track_state` / `age_s` / `valid_until` / `quality`（见 [目标跟踪](#目标跟踪)）。

---

## 参数

### 检测后端

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `detector_backend` | `hsv` | `hsv` = OpenCV 颜色阈值方案；`yolo` = ONNX Runtime 推理 |
| `model_path` | `""` | ONNX 模型路径（`yolo` 后端必填） |
| `model_conf_threshold` | `0.25` | 置信度阈值 |
| `model_iou_threshold` | `0.7` | NMS 的 IoU 阈值 |
| `model_input_size` | `640` | 模型输入边长，必须与模型一致 |
| `model_num_threads` | `0` | ORT 推理线程数，`0` = 自动；核多的机器试 `4` |
| `model_class_names` | `green_buoy` | 类别名，多个用逗号分隔 |

> 两个后端产出的 `Detection` 结构完全一致（`class_name` / `center` / `bbox` / `score`，均为原图像素坐标），
> 下游无需关心用的是哪个。**`class_name` 统一为 `green_buoy`**（与数据集 `classes.txt` 一致），
> `yolo` 后端从 `model_class_names` 读取，`hsv` 后端固定填同一个值。

### 检测参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `h_min` / `h_max` | `35` / `85` | HSV 色相范围（OpenCV 中 H 为 0~180） |
| `s_min` / `s_max` | `75` / `255` | 饱和度范围 |
| `v_min` / `v_max` | `41` / `255` | 明度范围 |
| `min_area` | `100` | 连通域最小面积（像素），小于则丢弃 |
| `kernel_size` | `8` | 形态学核边长，`Size(k, k)` 椭圆核 |
| `adaptive_block_size` | `11` | 自适应阈值窗口，偶数会自动 +1 |
| `adaptive_C` | `2.0` | 自适应阈值常数 C（越小越敏感） |
| `stale_threshold_sec` | `0.5` | 数据年龄阈值（秒），超过判为 `INPUT_STALE` |
| `publish_debug_mask` | `true` | 是否发布 `debug_mask`（二值 mask） |
| `publish_debug_image` | `true` | 是否发布 `debug`（检测框标注图） |

### 异常样本保存参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `save_failure_samples` | `false` | 是否保存（仅 `INPUT_STALE` / `NO_DETECTION` / `CONFIG_ERROR` 三类） |
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

构建时会打印是否找到 ONNX Runtime：

```text
找到 ONNX Runtime: /home/.../lib/libonnxruntime.so  → 启用 YOLO 后端
# 或者
未找到 ONNX Runtime → YOLO 后端不可用（HSV 不受影响）
```

**找不到也不影响构建**，`yolo` 后端会在运行时返回 `STATUS_MODEL_ERROR` 并说明原因。
需要装时下载 linux-x64 的 tgz 解压即可，然后：

```bash
colcon build --packages-select moonwake_camera \
  --cmake-args -DONNXRUNTIME_DIR=$HOME/third_party/onnxruntime
```

---

## 运行

### 一条命令跑完整链路（推荐）
以我电脑上的绝对路径为例

```bash
ros2 launch moonwake_camera test_with_video.launch.py \
  video:=/home/shaofei/Videos/test.mp4 rate:=10.0
```

同时启动「视频发布 + 检测 + 评估」，`Ctrl+C` 后自动打印统计报告并保存 CSV。

如果参数文件里 `publish_debug_image: true`，**还会自动弹出一个 rqt_image_view 窗口**显示 `/debug`，
不用再另开终端。不想开窗口可以加 `show_debug_image:=false`。

默认用 `hsv` 后端。换 YOLO 后端加两个参数即可：

```bash
ros2 launch moonwake_camera test_with_video.launch.py \
  video:=/home/shaofei/Videos/test2.mp4 rate:=5.0 \
  detector_backend:=yolo \
  model_path:=/home/shaofei/perception_release_v1/model.onnx
```

> ⚠️ YOLO 在 CPU 上单帧约 **110~230 ms**（取决于线程数设置），跑不到 10 FPS，
> 用 YOLO 时把 `rate` 设成 **4~8**，否则队列会堆积、帧全被判成 `INPUT_STALE`。

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video` | `''` | 视频路径；留空则跳过发布节点（接真相机时用） |
| `rate` | `0.0` | 发布帧率，0 = 跟随视频自身 FPS（推荐）；想放慢可填更小值 |
| `loop` | `true` | 播完是否循环 |
| `params_file` | 包内 `config/detector_params.yaml` | 参数文件 |
| `detector_backend` | `''` | 覆盖检测后端（`hsv` / `yolo`）；留空用 yaml 里的值 |
| `model_path` | `''` | 覆盖 ONNX 模型路径；留空用 yaml 里的值 |
| `show_debug_image` | `auto` | 是否自动打开 `/debug` 窗口：`auto` = 跟随 yaml 里的 `publish_debug_image`；也可填 `true` / `false` |
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

## 检测后端

节点内置两个检测后端，由参数 `detector_backend` 选择，接口统一为 `IDetector::detect(bgr)`：

| 后端 | 原理 | 依赖 | `class_name` |
| --- | --- | --- | --- |
| `hsv`（默认） | HSV 颜色阈值 + 自适应阈值 + 连通域 | OpenCV | 固定 `buoy` |
| `yolo` | YOLO11 ONNX 推理 + NMS | ONNX Runtime（可选） | 取 `model_class_names` |

```bash
# 用 hsv（默认）
ros2 launch moonwake_camera test_with_video.launch.py video:=/path/test.mp4

# 用 yolo（launch 直接覆盖后端）
ros2 launch moonwake_camera test_with_video.launch.py \
  video:=/path/test.mp4 rate:=5.0 \
  detector_backend:=yolo \
  model_path:=$HOME/perception_release_v1/model.onnx

# 也可以单独跑节点时用 -p 覆盖
ros2 run moonwake_camera buoy_detector_node --ros-args \
  --params-file ~/moonwake_autonomy/src/moonwake_camera/config/detector_params.yaml \
  -p detector_backend:=yolo -p model_path:=$HOME/perception_release_v1/model.onnx
```

**ONNX Runtime 是可选的**：编译时找不到也不影响构建，`yolo` 后端会在运行时返回
`STATUS_MODEL_ERROR` 并说明原因（`hsv` 后端完全不受影响）。

```bash
# 装 ONNX Runtime（下载 linux-x64 的 tgz 解压到 ~/third_party/onnxruntime 即可，CMake 会自动找）
colcon build --packages-select moonwake_camera \
  --cmake-args -DONNXRUNTIME_DIR=$HOME/third_party/onnxruntime
```

YOLO 前后处理与 `perception_release_v1/preprocessing.md` 的契约保持一致：

```text
BGR → letterbox 640（等比缩放，灰边 114，居中）
    → blobFromImage(scale=1/255, swapRB=true)
    → forward → [1, 4+nc, 8400] 转置
    → conf 过滤 + NMSBoxes(iou=0.7)
    → 坐标还原：先减 pad，再除 scale
```

---

## 目标跟踪

检测器每帧只说“这儿有个框”，不知道“这是不是上一帧那个”。跟踪器补上**身份**，
让下游能说“1 号浮标往左了”，而不是“又冒出来一个浮标”。

实现直接写在 `buoy_detector_node.cpp` 里（`KalmanTrack` + `update_tracks()`），没有单独节点。

### 一帧里的顺序

```text
① 全部轨迹先预测（卡尔曼 predict）
② 检测框 → 几何框
③ 数据关联：贪心匹配 + 三重门限
④ 没配上的轨迹 → misses++，已确认的发布一条 predicted_tracks（TRACK_LOST）
⑤ 没配上的检测 → 新建轨迹（TENTATIVE）
⑥ 把 track_id / age_s / valid_until / quality 写回 detections
⑦ 淘汰：误检快速删，真丢失等 max_lost_s 才删
```

### 状态模型

8 维卡尔曼状态，单位是**秒**（`dt` 由 `header.stamp` 计算，与帧率无关）：

```text
x = [cx, cy, w, h, vx, vy, vw, vh]      观测 z = [cx, cy, w, h]
```

速度不是直接测出来的，是从“位置随时间的变化”推出来的。

### 对外字段

| 字段 | 含义 |
| --- | --- |
| `track_id` | 跨帧稳定的编号，只增不减；`-1` 表示没有跟踪信息（`track_enable:=false`） |
| `track_state` | `TRACK_TENTATIVE`(0) 刚出现 / `TRACK_CONFIRMED`(1) 已确认 / `TRACK_LOST`(2) 只有预测 |
| `age_s` | 这条轨迹已经存在多久（秒） |
| `valid_until` | 观测有效期（ROS 时间秒）。**下游必须检查**：超过就该丢弃 |
| `quality` | 0~1 质量分 = `命中/(命中+丢失) × min(1, 命中/min_hits)` |

> `predicted_tracks` 里的框**不是观测**：位置来自卡尔曼预测，`score` 等于 `quality`。
> 下游只要真实观测的话，可以完全忽略这个数组。

### 生命周期与两个时间点

```text
TENTATIVE ──连续命中 min_hits 次──► CONFIRMED
                                      │
                   超过 max_age_s 没见到│（停止发布 predicted_tracks）
                                      ▼
                                    LOST
                                      │
                    超过 max_lost_s 没见到│（删除轨迹）
                                      ▼
                                   DELETED
```

**“停止发布”和“删除”是两个不同的时间点**，中间那段“沉默期”专门用来在目标重新出现时接回原 ID。

### 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `track_enable` | `true` | 总开关；关掉后 `track_id` 恒为 `-1`，行为与加跟踪前完全一致 |
| `track_min_hits` | `3` | 连续命中几次才确认 |
| `track_max_age_s` | `0.5` | 超过这么多秒没见到 → 停止发布预测 |
| `track_max_lost_s` | `3.0` | 超过这么多秒没见到 → 删除轨迹（调大可容忍更久遮挡） |
| `track_iou_gate` | `0.02` | IoU 低于此值不许配对（半遮挡时框会缩水，所以只要求“有一点点重叠”） |
| `track_dist_gate` | `150.0` | 距离门限的**基数**（px） |
| `track_dist_gate_rate` | `120.0` | 每丢失 1 秒，半径放宽多少（px/s） |
| `track_dist_gate_max` | `400.0` | 半径**天花板**（px），防止画面切换时乱配 |
| `track_size_ratio_min` / `max` | `0.5` / `2.0` | 新框宽 / 旧框宽 必须落在这个区间 |

实际使用的距离门限：

```text
允许距离 = min(track_dist_gate_max, track_dist_gate + track_dist_gate_rate × 丢失秒数)
```

丢得越久，预测越不可信，搜索半径越大；但**必须有上限**，否则任何检测都会被硬接到老轨迹上。

### 为什么门限要随时间放宽

固定门限在物理上说不通：刚丢 0.1 秒时预测很准，门限该严；丢了 2.5 秒时预测漂得厉害，门限该松。
用同一个常数，必然一边太松一边太紧——**遮挡 1~2 秒后 ID 就会断**。

### 调参建议

| 症状 | 先调这个 |
| --- | --- |
| 遮挡后 ID 断开 | `track_max_lost_s` 调大；或 `track_dist_gate_rate` 调大 |
| 画面切换/误检被接到老轨迹上 | `track_dist_gate_max` 调小 |
| ID 频繁跳变 | `track_iou_gate` 调小、`track_dist_gate` 调大 |
| 一个目标出现两条轨迹 | `track_iou_gate` 调大 |
| 误检也占了 ID | `track_min_hits` 调大 |

`track_max_lost_s` 不要拍脑袋：量一批实际视频里“浮标被遮挡的时长”，取 **P95** 再乘 1.2。

### 怎么验证跟踪是否正常

| 测试 | 期望 |
| --- | --- |
| 正常跟拍 | 一段视频里只有 1 个 `track_id`；`age_s` 一直增长 |
| 短时遮挡（< `max_lost_s`） | 遮挡期间出现橙色 `P<id>` 预测框；恢复后 **ID 不变** |
| 超过 `max_lost_s` | `predicted_tracks` 变空（目标失效），恢复后是新 ID |
| 快速转向 | ID 不应频繁跳变 |

`/debug` 图里：**绿色框 + `ID3`** 是真实观测，**橙色框 + `P3 q0.6`** 是纯预测（带质量分）。

---

## 测试流程

### 1. 链路冒烟测试（30 秒跑通）

```bash
source /opt/ros/humble/setup.bash
source ~/moonwake_autonomy/install/setup.bash

ros2 launch moonwake_camera test_with_video.launch.py \
  video:=/home/shaofei/Videos/test2.mp4 rate:=10.0
```

`publish_debug_image: true` 时窗口会**自动弹出**（显示 `/debug`，右上角有实时 FPS）。
如果没弹出来，可以手动开：

```bash
ros2 run rqt_image_view rqt_image_view /debug
```

**期望**：评估节点每 5 秒打一行进度；`/debug` 里能看到绿色检测框；
`Ctrl+C` 后打印统计报告并生成 `eval_result.csv`。

### 2. 切换后端

| 目标 | 做法 |
| --- | --- |
| `hsv`（默认） | 不加参数 |
| `yolo` | 加 `detector_backend:=yolo model_path:=<模型路径>` |

确认方式：启动日志里有一行 `detector backend: hsv` / `detector backend: yolo`。

### 3. 收集指标

```bash
ros2 launch moonwake_camera test_with_video.launch.py \
  video:=/path/test.mp4 rate:=10.0 eval_csv:=run_hsv.csv
```

跑完后 CSV 里每帧一行（`Timestamp,Status,Process_Time_ms,Data_Age_ms,Num_Detections`），
可以直接用 pandas / Excel 分析检出率和耗时分布。

### 4. 调参闭环（只对 hsv 后端有意义）

```text
失败样本 failure_samples/  →  tune_params.py 调参  →  写进 detector_params.yaml
        ↑                                                    │
        └──────── 重新跑测试、看 eval CSV ←── 重新构建 + 启动 ←┘
```

### 5. 常见现象对照

| 现象 | 多半是 |
| --- | --- |
| `ros2 node list` 里没有 `/buoy_detector_node` | 启动就崩了，看 launch 日志里的 `process has died` |
| 状态全是 `INPUT_STALE` | `rate` 超过后端处理能力（YOLO 尤其明显），或时间戳不是"当前时间" |
| 状态全是 `MODEL_ERROR` | 模型路径错 / 未安装 ONNX Runtime / 模型文件损坏 |
| 状态全是 `NO_DETECTION` | hsv：阈值或 `min_area` 不合适；yolo：`model_conf_threshold` 太高 |
| 有 `/debug` 没有 `/debug_mask` | `yolo` 后端不产生 mask，属正常 |

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
- `yolo` 后端在 CPU 上单帧约 110~230 ms（`model_num_threads` 影响很大），实时性受限；
  想上 30 FPS 需要降输入尺寸、换 OpenVINO/CUDA EP，或上 GPU。
- `debug_mask` 只有 `hsv` 后端产生，`yolo` 后端不发布该话题。

### 跟踪相关

- **纯几何关联**：只用了 IoU / 中心距离 / 尺寸一致性，没有外观特征（ReID）。
  如果两个目标的位置和大小都很接近（例如两个浮标并排），无法可靠区分。
- **门限是近似**：用“随丢失时间线性放宽 + 天花板”近似卡尔曼协方差增长，
  不是严格的马氏距离门控。实现简单、好调，但在长时间遮挡后不是最优。
- **`Q` / `R` 是经验值**，尚未用实船数据标定。`R` 假设检测框抖动约 ±5 px，
  `Q` 按“匀速模型 + 船体晃动”估。后续应实测：静止目标测 `R`，用跟踪误差反推 `Q`。
- **未做多目标交叉测试**：当前数据集每张图只有一个浮标，两目标交叉、互相遮挡的场景尚未验证。
- **`track_id` 的连续性依赖参数**：`track_max_lost_s` 设得比实际遮挡时长短，就会换 ID（这是设计行为，不是 bug）。
- **时间戳依赖**：所有超时判断都用 `header.stamp`。若发布端送的是当前时间（`video_publisher.py` 默认如此），
  用 `ros2 bag play` 回放时需加 `use_sim_time:=true`，否则轨迹会被瞬间判超时。
