#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
evaluate_detector.py —— 视觉检测节点性能评估工具
=================================================

订阅 DetectionArray 话题，逐帧写 CSV，退出时打印汇总报告。

用法
----
    # 1) 直接运行（需要先 source 环境）
    source /opt/ros/humble/setup.bash
    source ~/moonwake_autonomy/install/setup.bash
    python3 ~/moonwake_autonomy/src/moonwake_camera/tools/evaluate_detector.py

    # 2) 指定话题（注意：检测节点实际发布的是 /detection，不是 /detections）
    python3 evaluate_detector.py --ros-args -p topic:=/detection

    # 3) 编译安装后也可以用 ros2 run
    ros2 run moonwake_camera evaluate_detector.py --ros-args -p topic:=/detection

参数
----
    topic          (str)     订阅的话题，默认 "detections"
    output_csv     (str)     CSV 路径，默认 "eval_result.csv"（当前目录）
    print_interval (double)  每几秒打印一次进度，<=0 关闭，默认 5.0
    qos_depth      (int)     订阅队列深度，默认 10

说明
----
* 状态名不是写死的：启动时从 DetectionArray 消息类里读 STATUS_* 常量，
  所以消息定义改了脚本也不用改。你需求文档里写的是 DETECTED=1、NO_DETECTION=2……，
  但实际 msg 里是 DETECTED=0、NO_DETECTION=1、INPUT_ERROR=2、INPUT_STALE=3、CONFIG_ERROR=4，
  这里以真实消息为准。
* CSV 的 Timestamp 列取的是图像 header.stamp（ROS 时间，单位秒），
  不是接收时刻；接收时刻你在分析时可以另外加。
* 处理耗时只统计 process_time_ms > 0 的帧（即真正跑过 OpenCV 的帧），
  INPUT_STALE / CONFIG_ERROR 那类提前 return 的帧耗时为 0，会把平均值拉低。
"""

import csv
import sys
import time
import unicodedata
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from moonwake_camera.msg import DetectionArray
except ImportError as exc:  # 没 source 工作区时给出明确提示，而不是一堆 traceback
    sys.exit(
        "导入 moonwake_camera.msg 失败。请先加载环境：\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source <你的工作区>/install/setup.bash\n"
        f"原始错误：{exc}"
    )


# ═════════════════════════════ 终端排版工具 ═════════════════════════
# 中文字符在终端里占 2 列，用 len() 算宽度会导致表格错位，
# 所以用 unicodedata 判断"东亚宽字符"。

def disp_width(text: str) -> int:
    """字符串在终端里实际占多少列。"""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
               for ch in text)


def pad(text: str, width: int, align: str = "left") -> str:
    """按显示宽度补空格。"""
    gap = max(0, width - disp_width(text))
    if align == "right":
        return " " * gap + text
    if align == "center":
        left = gap // 2
        return " " * left + text + " " * (gap - left)
    return text + " " * gap


def safe_div(a: float, b: float) -> float:
    """除零保护：总帧数为 0 时统计函数不能炸。"""
    return a / b if b else 0.0


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time -> 秒（float）。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# ═════════════════════════════ 统计数据 ═════════════════════════════

class EvalStats:
    """所有统计量集中在这里，回调里只调 add()，汇总时只读字段。"""

    def __init__(self):
        self.total_frames = 0
        self.status_counts = Counter()        # 状态名 -> 帧数
        self.total_detections = 0             # 累加每帧 len(detections)
        self.frames_with_detections = 0       # 至少检测到 1 个目标的帧数

        # 处理耗时：只统计 > 0 的帧
        self.proc_sum = 0.0
        self.proc_max = 0.0
        self.proc_min = float("inf")
        self.proc_frames = 0

        # 数据延迟：所有帧都统计
        self.age_sum = 0.0
        self.age_max = 0.0

        # 运行时间（用接收时刻的单调时钟，不受系统时间调整影响）
        self.first_recv = None
        self.last_recv = None

    def add(self, status_name, process_time_ms, data_age_ms, num_dets, recv_time):
        self.total_frames += 1
        self.status_counts[status_name] += 1
        self.total_detections += num_dets
        if num_dets > 0:
            self.frames_with_detections += 1

        if process_time_ms > 0.0:
            self.proc_sum += process_time_ms
            self.proc_max = max(self.proc_max, process_time_ms)
            self.proc_min = min(self.proc_min, process_time_ms)
            self.proc_frames += 1

        self.age_sum += data_age_ms
        self.age_max = max(self.age_max, data_age_ms)

        if self.first_recv is None:
            self.first_recv = recv_time
        self.last_recv = recv_time

    @property
    def duration(self) -> float:
        if self.first_recv is None or self.last_recv is None:
            return 0.0
        return max(0.0, self.last_recv - self.first_recv)

    @property
    def avg_proc(self) -> float:
        return safe_div(self.proc_sum, self.proc_frames)

    @property
    def avg_age(self) -> float:
        return safe_div(self.age_sum, self.total_frames)

    @property
    def fps(self) -> float:
        return safe_div(self.total_frames, self.duration)

    @property
    def avg_dets(self) -> float:
        return safe_div(self.total_detections, self.total_frames)

    @property
    def avg_dets_when_found(self) -> float:
        return safe_div(self.total_detections, self.frames_with_detections)


# ═════════════════════════════ 报告渲染 ═════════════════════════════

def render_report(stats: EvalStats, topic: str, csv_path: Path, status_map: dict) -> str:
    """把统计结果排版成一块易读的报告。"""
    width = 68
    out = []
    out.append("═" * width)
    out.append(pad("moonwake_camera 检测性能评估报告", width, "center"))
    out.append("═" * width)

    out.append(f" 话题        : {topic}")
    out.append(f" 输出 CSV    : {csv_path}")
    out.append(f" 运行时长    : {stats.duration:.2f} s")

    # ── 帧统计 ──
    out.append("─" * width)
    out.append(" 【帧统计】")
    out.append(f"   总帧数            : {stats.total_frames}")
    out.append(f"   平均帧率          : {stats.fps:.2f} FPS")
    out.append(f"   有效处理帧        : {stats.proc_frames}"
               f"   (未进入检测的帧 {stats.total_frames - stats.proc_frames})")

    # ── 状态分布 ──
    out.append("─" * width)
    out.append(" 【状态分布】")
    if stats.total_frames == 0:
        out.append("   (没有收到任何消息)")
    else:
        # 已知状态按状态码排序，未知状态追加在后面
        known = [status_map.get(k, f"STATUS_{k}") for k in sorted(status_map)]
        seen = list(stats.status_counts.keys())
        order = [n for n in known if n in seen] + [n for n in seen if n not in known]
        bar_max = 18
        for name in order:
            count = stats.status_counts[name]
            ratio = safe_div(count, stats.total_frames)
            bar = "█" * int(round(ratio * bar_max))
            out.append(f"   {pad(name, 22)} {count:>7d}  {ratio * 100:>6.2f}%  {bar}")

    # ── 时间指标 ──
    out.append("─" * width)
    out.append(" 【时间指标】")
    if stats.proc_frames:
        out.append(f"   处理耗时 ms        : 平均 {stats.avg_proc:7.2f}"
                   f"   最大 {stats.proc_max:7.2f}   最小 {stats.proc_min:7.2f}")
    else:
        out.append("   处理耗时 ms        : 无有效数据（没有帧真正进入检测流程）")
    out.append(f"   数据延迟 ms        : 平均 {stats.avg_age:7.2f}   最大 {stats.age_max:7.2f}")

    # ── 目标统计 ──
    out.append("─" * width)
    out.append(" 【目标统计】")
    out.append(f"   检测到的目标总数  : {stats.total_detections}")
    out.append(f"   平均每帧目标数    : {stats.avg_dets:.2f}")
    out.append(f"   有目标帧的平均数  : {stats.avg_dets_when_found:.2f}"
               f"   (有目标帧 {stats.frames_with_detections})")
    out.append("═" * width)
    return "\n".join(out)


# ═════════════════════════════ 评估节点 ═════════════════════════════

class DetectorEvaluator(Node):
    """订阅检测结果，累计统计并写 CSV。"""

    def __init__(self):
        super().__init__("detector_evaluator")

        # ---------- 参数 ----------
        self.declare_parameter("topic", "detections")
        self.declare_parameter("output_csv", "eval_result.csv")
        self.declare_parameter("print_interval", 5.0)
        self.declare_parameter("qos_depth", 10)

        self.topic = str(self.get_parameter("topic").value)
        self.print_interval = float(self.get_parameter("print_interval").value)
        depth = int(self.get_parameter("qos_depth").value)

        # ---------- 状态码 -> 名称（从消息类动态读取，不写死） ----------
        self.status_map = {}
        for attr in dir(DetectionArray):
            if not attr.startswith("STATUS_"):
                continue
            value = getattr(DetectionArray, attr)
            if isinstance(value, int):
                self.status_map[int(value)] = attr

        if self.status_map:
            mapping = ", ".join(f"{v}={n}" for v, n in sorted(self.status_map.items()))
            self.get_logger().info(f"状态码映射（来自消息定义）: {mapping}")
        else:
            self.get_logger().warning("消息类里没找到 STATUS_* 常量，状态名将显示为 STATUS_<数字>")

        # ---------- CSV ----------
        self.csv_path = Path(str(self.get_parameter("output_csv").value)).expanduser().resolve()
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            # newline="" 是 csv 模块的硬性要求，否则 Windows 下会多空行
            self._csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        except OSError as exc:
            raise RuntimeError(f"无法创建 CSV 文件 {self.csv_path}: {exc}") from exc

        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(
            ["Timestamp", "Status", "Process_Time_ms", "Data_Age_ms", "Num_Detections"])
        self._csv_file.flush()
        self._last_flush = time.monotonic()

        # ---------- 统计 ----------
        self.stats = EvalStats()
        self._checked_topic = False

        # ---------- 订阅 ----------
        # 检测节点用的是默认 QoS（RELIABLE），这里保持一致，避免"看不到消息"
        qos = QoSProfile(
            depth=depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(
            DetectionArray, self.topic, self.on_message, qos)

        self.get_logger().info(f"开始订阅 {self.topic}，统计结果将写入 {self.csv_path}")

        if self.print_interval > 0:
            self.create_timer(self.print_interval, self.on_timer)

    # ---------------- 回调 ----------------

    def on_message(self, msg):
        """每收到一帧：更新统计 + 追加一行 CSV。"""
        recv_time = time.monotonic()
        status_name = self.status_map.get(int(msg.status), f"STATUS_{int(msg.status)}")
        num_dets = len(msg.detections)

        self.stats.add(
            status_name=status_name,
            process_time_ms=float(msg.process_time_ms),
            data_age_ms=float(msg.data_age_ms),
            num_dets=num_dets,
            recv_time=recv_time,
        )

        self._writer.writerow([
            f"{stamp_to_sec(msg.header.stamp):.6f}",
            status_name,
            f"{float(msg.process_time_ms):.3f}",
            f"{float(msg.data_age_ms):.3f}",
            num_dets,
        ])

        # 每写一行都 flush 太伤磁盘（高频下每秒上百次写入），
        # 这里限制成最多每秒刷一次；退出时还会再 flush 一次，不会丢数据。
        now = time.monotonic()
        if now - self._last_flush >= 1.0:
            self._csv_file.flush()
            self._last_flush = now

    def on_timer(self):
        """周期性进度输出 + 首个周期做一次"话题是否正确"的自检。"""
        if not self._checked_topic:
            self._checked_topic = True
            self._check_topic()

        if self.stats.total_frames == 0:
            return
        s = self.stats
        self.get_logger().info(
            f"帧 {s.total_frames:>5d} | "
            f"检测到 {s.status_counts.get('STATUS_DETECTED', 0):>5d} | "
            f"无目标 {s.status_counts.get('STATUS_NO_DETECTION', 0):>5d} | "
            f"平均耗时 {s.avg_proc:6.2f} ms | "
            f"平均延迟 {s.avg_age:7.2f} ms"
        )

    def _check_topic(self):
        """没消息时，帮用户确认是不是话题名写错了。"""
        if self.stats.total_frames > 0:
            return

        publishers = self.count_publishers(self.topic)
        if publishers > 0:
            self.get_logger().warning(
                f"{self.topic} 上有 {publishers} 个发布者，但还没收到消息"
                f"（可能 QoS 不匹配，或对方还没开始发）")
            return

        # 找找看还有哪些 DetectionArray 话题（排除自己刚建的那个订阅）
        own = self.subscription.topic_name          # 已解析成全名，如 /detections
        candidates = [name for name, types in self.get_topic_names_and_types()
                      if name != own and "DetectionArray" in " ".join(types)]
        msg = f"{own} 上没有发布者，也没收到消息。"
        if candidates:
            msg += f" 发现这些 DetectionArray 话题: {candidates}，"
            msg += f"可以用 -p topic:={candidates[0]} 重跑"
        else:
            msg += " 请确认检测节点已启动，并用 ros2 topic list 检查话题名"
        self.get_logger().warning(msg)

    # ---------------- 收尾 ----------------

    def close(self):
        """确保 CSV 被刷盘并关闭（幂等，重复调用安全）。"""
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # 退出路径上不要再抛异常打断汇总打印
            self.get_logger().error(f"关闭 CSV 时出错: {exc}")

    def summary(self) -> str:
        return render_report(self.stats, self.topic, self.csv_path, self.status_map)


# ═════════════════════════════ 主程序 ═════════════════════════════

def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    exit_code = 0

    try:
        node = DetectorEvaluator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 是正常退出路径，不是错误
        print("\n收到 Ctrl+C，正在汇总……")
    except ExternalShutdownException:
        # 外部关掉了 context（比如 launch 发的 SIGTERM）
        pass
    except RuntimeError as exc:  # CSV 打不开之类
        print(f"启动失败: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            print()
            print(node.summary())
            print(f"CSV 已保存: {node.csv_path}  （{node.stats.total_frames} 行数据）")
            # ros2 launch 收到 Ctrl+C 时会先把 rclpy context 关掉，
            # 这时再 destroy_node() 会抛异常（看起来像崩溃，其实是误报）；
            # 退出过程中还可能再收到一次 SIGINT，所以这里抛出的可能是 KeyboardInterrupt。
            # 注意：KeyboardInterrupt 继承自 BaseException 而不是 Exception，
            # 写成 except Exception 抓不住，必须用 BaseException。
            # CSV 已经在 close() 里落盘，这里吞掉异常即可。
            try:
                node.destroy_node()
            except BaseException:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
