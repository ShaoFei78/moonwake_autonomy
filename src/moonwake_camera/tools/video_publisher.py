#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_publisher.py —— 用视频文件代替真实相机
=============================================

读一段视频，按固定帧率发布 image_raw + camera_info，
让 buoy_detector_node 像接真相机一样工作。

为什么需要它
------------
你的节点用 message_filters::ApproximateTime 同步 image_raw 和 camera_info，
所以这两个话题必须同时有、且 header.stamp 要一致——本脚本就是这么发的。

最关键的一点：时间戳用的是「当前 ROS 时间」，不是视频里的原始时间。
如果直接把视频的原始 PTS 塞进 header.stamp，节点算出来的数据年龄会是几秒甚至几分钟，
每帧都会被判成 STATUS_INPUT_STALE，你会以为算法坏了。

用法
----
    # 基本用法
    ros2 run moonwake_camera video_publisher.py --ros-args -p video:=/home/shaofei/test.mp4

    # 指定帧率（默认跟随视频自身 FPS）
    ros2 run moonwake_camera video_publisher.py --ros-args \
        -p video:=/home/shaofei/test.mp4 -p rate:=15.0

    # 播完不循环，停在最后一帧
    ... -p loop:=false

参数
----
    video      (str)   视频文件路径（必填）。支持 mp4/avi/mkv 等 OpenCV 能解的格式
    rate       (double) 发布帧率；<=0 表示沿用视频自身 FPS（读不到就用 30）
    loop       (bool)  播完是否从头循环，默认 true
    frame_id   (str)   header.frame_id，默认 "camera"
    qos_depth  (int)   发布队列深度，默认 10

配合使用
--------
    终端 1：
        ros2 run moonwake_camera video_publisher.py --ros-args -p video:=/path/test.mp4

    终端 2：
        ros2 run moonwake_camera buoy_detector_node --ros-args \
          --params-file ~/moonwake_autonomy/src/moonwake_camera/config/detector_params.yaml

    终端 3（看画面）：
        ros2 run rqt_image_view rqt_image_view /image_raw
        # 或看 mask： ros2 run rqt_image_view rqt_image_view /debug_mask

    终端 4（评估）：
        ros2 run moonwake_camera evaluate_detector.py --ros-args -p topic:=/detection
"""

import sys

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class VideoPublisher(Node):
    """把视频文件当成一台"帧率固定、时间戳实时"的相机。"""

    def __init__(self):
        super().__init__("video_publisher")

        self.declare_parameter("video", "")
        self.declare_parameter("rate", 0.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("qos_depth", 10)

        video_path = str(self.get_parameter("video").value)
        if not video_path:
            raise RuntimeError(
                "必须指定视频文件，例如：-p video:=/home/shaofei/test.mp4")

        self.loop = bool(self.get_parameter("loop").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        depth = int(self.get_parameter("qos_depth").value)

        # ---------- 打开视频 ----------
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"打不开视频文件：{video_path}\n"
                f"确认路径正确，且 OpenCV 支持该编码格式（可先用 ffmpeg 转成 mp4/H.264）")

        video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        rate_param = float(self.get_parameter("rate").value)
        if rate_param > 0:
            self.rate = rate_param
        else:
            self.rate = video_fps if video_fps and video_fps > 0 else 30.0

        # 读一帧拿到分辨率，然后把读指针拨回开头，保证从第 0 帧开始播
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"视频里读不到任何帧：{video_path}")
        self.height, self.width = frame.shape[:2]
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

        # ---------- 发布者 ----------
        # 和检测节点的默认 QoS 保持一致（RELIABLE），避免订阅端收不到
        qos = QoSProfile(
            depth=depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.image_pub = self.create_publisher(Image, "image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", qos)

        # ---------- 内参模板 ----------
        # 你的检测节点目前只校验 width/height 是否和 CameraInfo 一致，
        # 内参数值暂时不参与计算。这里填一组"看起来合理"的针孔模型参数，
        # 以后要做去畸变/测距时再换成真实标定值。
        self.camera_info = CameraInfo()
        self.camera_info.width = self.width
        self.camera_info.height = self.height
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = fy = float(self.width)          # 粗略假设：焦距 ≈ 图像宽度
        cx, cy = self.width / 2.0, self.height / 2.0
        self.camera_info.k = [fx, 0.0, cx,
                              0.0, fy, cy,
                              0.0, 0.0, 1.0]
        self.camera_info.r = [1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0]
        self.camera_info.p = [fx, 0.0, cx, 0.0,
                              0.0, fy, cy, 0.0,
                              0.0, 0.0, 1.0, 0.0]

        self.frames_sent = 0
        self._finished = False

        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)

        self.get_logger().info(
            f"视频: {video_path}")
        self.get_logger().info(
            f"分辨率 {self.width}x{self.height}，"
            f"总帧数 {total if total > 0 else '未知'}，"
            f"视频 FPS {video_fps:.2f}，按 {self.rate:.2f} FPS 发布，"
            f"loop={'on' if self.loop else 'off'}")

    # ------------------------------------------------------------

    def on_timer(self):
        if self._finished:
            return

        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                # 回到开头重播
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            if not ok:
                self._finished = True
                self.get_logger().info(
                    f"视频播放结束，共发布 {self.frames_sent} 帧")
                self.destroy_timer(self.timer)
                return

        # 时间戳用「现在」——这是整个脚本最关键的一行
        stamp = self.get_clock().now().to_msg()

        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = self.width * 3
        img_msg.data = frame.tobytes()

        # 两个话题的时间戳必须一致，否则 ApproximateTime 匹配不上
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id

        self.image_pub.publish(img_msg)
        self.info_pub.publish(self.camera_info)
        self.frames_sent += 1

        # 每 5 秒报一次进度，方便确认确实在发
        if self.frames_sent % max(1, int(self.rate * 5)) == 0:
            self.get_logger().info(f"已发布 {self.frames_sent} 帧")

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


# ═════════════════════════════ 主程序 ═════════════════════════════

def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = VideoPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"启动失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            print(f"共发布 {node.frames_sent} 帧")
            # 同 evaluate_detector：launch 的 SIGINT 会先关 context，
            # 且退出中可能再收到一次 SIGINT（KeyboardInterrupt 属于 BaseException），
            # 两种都吞掉，避免打印一堆假崩溃
            try:
                node.destroy_node()
            except BaseException:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
