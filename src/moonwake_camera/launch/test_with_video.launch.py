#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_with_video.launch.py —— 一句命令跑完整条验证链路
=====================================================

启动三个节点：
    1. video_publisher   把视频文件当成相机，发 image_raw + camera_info
    2. buoy_detector_node 你的检测节点，用 config/detector_params.yaml 里的参数
    3. evaluate_detector.py 统计性能并写 CSV

用法
----
    # 最常用：指定一段视频
    ros2 launch moonwake_camera test_with_video.launch.py \
      video:=/home/shaofei/videos/test.mp4

    # 640x480 的视频建议显式限速到 10 FPS（Python 发布端跑不满 20）
    ros2 launch moonwake_camera test_with_video.launch.py \
      video:=/home/shaofei/videos/test.mp4 rate:=10.0

    # 不指定 video：跳过发布节点，只起检测 + 评估（接真相机时用）
    ros2 launch moonwake_camera test_with_video.launch.py

    # 换一套参数文件
    ros2 launch moonwake_camera test_with_video.launch.py \
      video:=/tmp/test.mp4 params_file:=/path/to/other.yaml

退出时
------
    按 Ctrl+C，评估节点会打印完整统计报告，
    CSV 默认落在「你执行 ros2 launch 时所在的目录」下的 eval_result.csv。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TRUE_SET = {"1", "true", "yes", "on"}


def _as_bool(text: str) -> bool:
    return text.strip().lower() in TRUE_SET


def _read_publish_debug_image(params_file: str) -> bool:
    """从参数文件里读出 publish_debug_image，读不到就当作 False。"""
    try:
        with open(params_file, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get("buoy_detector_node", {}).get("ros__parameters", {})
        return bool(params.get("publish_debug_image", False))
    except Exception as exc:      # 文件读不了/格式错，都不该让 launch 挂掉
        print(f"[launch] 读取 {params_file} 失败，跳过 debug 图像窗口: {exc}")
        return False


def _launch_setup(context):
    """
    真正创建节点的函数。

    放在 OpaqueFunction 里而不是直接写在 generate_launch_description() 里，
    是因为要根据 video 是否为空来决定要不要启动发布节点——
    这种"运行时才能确定"的逻辑必须在这里做。
    """
    video = LaunchConfiguration("video").perform(context).strip()
    rate = float(LaunchConfiguration("rate").perform(context))
    loop = _as_bool(LaunchConfiguration("loop").perform(context))
    params_file = os.path.expanduser(
        LaunchConfiguration("params_file").perform(context).strip())
    eval_enabled = _as_bool(LaunchConfiguration("start_evaluator").perform(context))
    eval_topic = LaunchConfiguration("eval_topic").perform(context)
    eval_csv = os.path.expanduser(
        LaunchConfiguration("eval_csv").perform(context).strip())
    print_interval = float(LaunchConfiguration("print_interval").perform(context))
    show_debug = LaunchConfiguration("show_debug_image").perform(context).strip().lower()
    # 空字符串 = 沿用参数文件里的值；非空 = 覆盖
    backend_override = LaunchConfiguration("detector_backend").perform(context).strip()
    model_path_override = os.path.expanduser(
        LaunchConfiguration("model_path").perform(context).strip())

    actions = []

    # ---------- 1. 视频发布 ----------
    if video:
        if not os.path.isfile(os.path.expanduser(video)):
            raise RuntimeError(f"找不到视频文件：{video}")
        actions.append(Node(
            package="moonwake_camera",
            executable="video_publisher.py",
            name="video_publisher",
            output="screen",
            # 先读参数文件（里面的 video_publisher 段提供 encoding），
            # 再用下面的 dict 覆盖视频路径等由 launch 参数决定的项
            parameters=[params_file, {
                "video": os.path.expanduser(video),
                "rate": rate,
                "loop": loop,
            }],
        ))
    else:
        actions.append(LogInfo(msg=(
            "未指定 video 参数，跳过视频发布节点；"
            "请确保有其它节点在发布 /image_raw 和 /camera_info")))

    # ---------- 2. 检测节点 ----------
    # 参数文件路径会原样传给节点；路径不对时 launch_ros 会直接报错，不会静默用默认值
    if not os.path.isfile(params_file):
        raise RuntimeError(f"找不到参数文件：{params_file}")
    detector_parameters = [params_file]
    overrides = {}
    if backend_override:
        overrides["detector_backend"] = backend_override
    if model_path_override:
        overrides["model_path"] = model_path_override
    if overrides:
        # 放在参数文件之后，同名的会覆盖文件里的值
        detector_parameters.append(overrides)

    actions.append(Node(
        package="moonwake_camera",
        executable="buoy_detector_node",
        name="buoy_detector_node",      # 必须和 yaml 顶层的 key 一致
        output="screen",
        parameters=detector_parameters,
    ))

    # ---------- 3. 评估 ----------
    if eval_enabled:
        actions.append(Node(
            package="moonwake_camera",
            executable="evaluate_detector.py",
            name="detector_evaluator",
            output="screen",
            parameters=[{
                "topic": eval_topic,
                "output_csv": eval_csv,
                "print_interval": print_interval,
            }],
        ))

    # ---------- 4. debug 图像窗口 ----------
    # show_debug_image 默认 auto：直接跟随参数文件里的 publish_debug_image，
    # 节点开标注图就顺手把窗口打开，省得再开一个终端
    if show_debug == "auto":
        show_window = _read_publish_debug_image(params_file)
    else:
        show_window = _as_bool(show_debug)

    if show_window:
        if os.environ.get("DISPLAY"):
            actions.append(ExecuteProcess(
                cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/debug"],
                output="screen",
            ))
        else:
            actions.append(LogInfo(msg=(
                "没有 DISPLAY 环境变量，跳过 rqt_image_view（无图形界面）")))

    return actions


def generate_launch_description():
    # 默认用包内安装的那份参数文件
    default_params = os.path.join(
        get_package_share_directory("moonwake_camera"),
        "config", "detector_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "video", default_value="",
            description="视频文件路径；留空则不启动发布节点（接真相机时用）"),
        DeclareLaunchArgument(
            "rate", default_value="0.0",
            description="发布帧率；0 表示跟随视频自身 FPS（推荐）。"
                        "想人为放慢就填个更小的值，例如 10.0"),
        DeclareLaunchArgument(
            "loop", default_value="true",
            description="播完是否循环"),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="检测节点的参数文件"),
        DeclareLaunchArgument(
            "detector_backend", default_value="",
            description="覆盖检测后端：hsv / yolo；留空则用参数文件里的值"),
        DeclareLaunchArgument(
            "model_path", default_value="",
            description="覆盖 ONNX 模型路径；留空则用参数文件里的值"),
        DeclareLaunchArgument(
            "show_debug_image", default_value="auto",
            description="是否自动打开 /debug 图像窗口："
                        "auto = 跟随参数文件里的 publish_debug_image；也可填 true / false"),
        DeclareLaunchArgument(
            "start_evaluator", default_value="true",
            description="是否启动评估节点"),
        DeclareLaunchArgument(
            "eval_topic", default_value="/detection",
            description="评估节点订阅的话题（注意节点实际发布的是 detection，单数）"),
        DeclareLaunchArgument(
            "eval_csv", default_value="eval_result.csv",
            description="评估 CSV 输出路径，相对路径基于执行 ros2 launch 的目录"),
        DeclareLaunchArgument(
            "print_interval", default_value="5.0",
            description="评估节点的进度打印间隔（秒），0 表示关闭"),

        # 所有节点在这里按运行参数动态创建
        OpaqueFunction(function=_launch_setup),
    ])
