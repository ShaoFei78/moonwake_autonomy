#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenCV 参数调试工具（滑动条版）
================================
流程和 buoy_detector_node.cpp 里的 sync_callback 完全一致，调的参数可以直接搬过去：

    C++                                       Python（本脚本）
    --------------------------------------    ----------------------------------
    cvtColor(cv_img, hsv, BGR2HSV)            cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    inRange(hsv, lower, upper, mask)          cv2.inRange(hsv, lower, upper)
    getStructuringElement(MORPH_ELLIPSE, k)   cv2.getStructuringElement(MORPH_ELLIPSE, (k, k))
    morphologyEx(OPEN) / (CLOSE)              cv2.morphologyEx(..., cv2.MORPH_OPEN / CLOSE)
    connectedComponentsWithStats              cv2.connectedComponentsWithStats
    area < min_area -> 丢弃                   area < min_area -> 丢弃
    bbox = [x, y, x+w, y+h]                   bbox = (x, y, x+w, y+h)
    score = area / (w*h)                      score = area / (w*h)

用法
----
    # 1) 带滑动条调参（最常用）
    python3 tune_params.py ~/failure_samples/xxx.png

    # 2) 不带参数：自动挑 <包目录>/failure_samples 里最新的那张原图
    python3 tune_params.py

    # 3) 看有哪些样本
    python3 tune_params.py --list

    # 4) 无界面批处理：用指定参数跑一遍，导出 mask（方便和 C++ 结果对比）
    python3 tune_params.py xxx.png --dump out_mask.png \
        -p h_min=0 -p h_max=10 -p s_min=120 -p s_max=255 \
        -p v_min=80 -p v_max=255 -p min_area=500 -p kernel_size=5

快捷键
------
    p      打印可直接复制的 ros2 run 参数
    s      保存当前 mask 和预览图到当前目录
    q/ESC  退出
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------- 参数定义

# 和节点里的 declare_parameter 默认值一一对应
DEFAULTS = {
    "h_min": 0,
    "h_max": 180,
    "s_min": 50,
    "s_max": 255,
    "v_min": 50,
    "v_max": 255,
    "min_area": 100,
    "kernel_size": 5,
}

# 滑动条的最大值
TB_MAX = {
    "h_min": 180, "h_max": 180,
    "s_min": 255, "s_max": 255,
    "v_min": 255, "v_max": 255,
    "min_area": 1000,   # 映射值，见 area_from_trackbar()
    "kernel_size": 31,
}

# min_area 用二次映射：滑动条 0~1000 -> 面积 0~100000
# 为什么不用线性？640x480 的图，目标面积一般几十到几万，
# 线性拖动的话小范围几乎调不动，二次映射让小值区间更精细。
def area_from_trackbar(v):
    return int(round(v * v / 10.0))


def trackbar_from_area(area):
    return int(round(math.sqrt(max(0, area) * 10.0)))


# ---------------------------------------------------------------- 核心流程

def build_params(values):
    """把 dict 里的一堆值整理成 OpenCV 需要的类型。"""
    return {
        "h_min": int(values["h_min"]),
        "h_max": int(values["h_max"]),
        "s_min": int(values["s_min"]),
        "s_max": int(values["s_max"]),
        "v_min": int(values["v_min"]),
        "v_max": int(values["v_max"]),
        "min_area": int(values["min_area"]),
        "kernel_size": int(values["kernel_size"]),
    }


def make_mask(bgr, p):
    """对应 C++ 的 cvtColor + inRange + getStructuringElement + OPEN + CLOSE。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([p["h_min"], p["s_min"], p["v_min"]], dtype=np.uint8)
    upper = np.array([p["h_max"], p["s_max"], p["v_max"]], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    k = max(1, p["kernel_size"])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_detections(mask, min_area):
    """对应 C++ 的 connectedComponentsWithStats + 面积过滤。"""
    n_labels, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

    dets = []
    for i in range(1, n_labels):  # 0 是背景，跳过
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        x = int(stats[i, cv2.CC_STAT_LEFT])
        y = int(stats[i, cv2.CC_STAT_TOP])
        w = int(stats[i, cv2.CC_STAT_WIDTH])
        h = int(stats[i, cv2.CC_STAT_HEIGHT])
        bbox_area = float(w * h)
        dets.append({
            "area": area,
            "center": (float(centroids[i][0]), float(centroids[i][1])),
            "bbox": (x, y, x + w, y + h),          # 和 C++ 一致：x1,y1,x2,y2
            "score": area / bbox_area if bbox_area > 0 else 0.0,
        })
    dets.sort(key=lambda d: d["area"], reverse=True)  # 大的排前面，方便看
    return dets, n_labels - 1


# ---------------------------------------------------------------- 显示

FONT = cv2.FONT_HERSHEY_SIMPLEX


def annotate(bgr, dets, p, total_components):
    vis = bgr.copy()
    for d in dets:
        x1, y1, x2, y2 = d["bbox"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cx, cy = int(d["center"][0]), int(d["center"][1])
        cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)
        label = f"a={d['area']} s={d['score']:.2f}"
        cv2.putText(vis, label, (x1, max(14, y1 - 6)),
                    FONT, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    # 左上角参数信息（OpenCV 的 putText 画不了中文，这里用英文）
    lines = [
        f"H {p['h_min']}-{p['h_max']}  S {p['s_min']}-{p['s_max']}  V {p['v_min']}-{p['v_max']}",
        f"min_area {p['min_area']}   kernel {p['kernel_size']}",
        f"components {total_components}   kept {len(dets)}",
    ]
    for i, line in enumerate(lines):
        y = 22 + i * 22
        cv2.putText(vis, line, (8, y), FONT, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, line, (8, y), FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def compose(original, annotated, mask):
    """左边原图+框，右边 mask，拼成一张方便对照。"""
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    panel = np.hstack([annotated, mask_bgr])

    max_width = 1400
    if panel.shape[1] > max_width:
        scale = max_width / panel.shape[1]
        panel = cv2.resize(panel, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
    return panel


# ---------------------------------------------------------------- 参数输出

def as_ros_args(p):
    keys = ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max",
            "min_area", "kernel_size"]
    return " ".join(f"-p {k}:={p[k]}" for k in keys)


def print_params(p, image_path=None):
    print()
    print("=" * 72)
    print("当前参数：")
    for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max",
              "min_area", "kernel_size"]:
        print(f"  {k:12s} = {p[k]}")
    print()
    print("可直接复制运行：")
    print(f"  ros2 run moonwake_camera buoy_detector_node --ros-args {as_ros_args(p)}")
    print()
    print("写进 yaml 的话：")
    print("  buoy_detector_node:")
    print("    ros__parameters:")
    for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max",
              "min_area", "kernel_size"]:
        print(f"      {k}: {p[k]}")
    print("=" * 72)


# ---------------------------------------------------------------- 找图片

def sample_dirs():
    """收集可能的样本目录，去重（cwd 就是家目录时两者会重合）。"""
    # 本脚本放在 <包目录>/tools/ 下，上两级就是包目录，
    # 也正是节点默认存样本的地方
    pkg_dir = Path(__file__).resolve().parent.parent
    dirs, seen = [], set()
    for d in (pkg_dir / "failure_samples",      # 节点默认位置
              Path.home() / "failure_samples",  # 旧默认位置，兼容一下
              Path.cwd() / "failure_samples"):  # 手动指定过的相对路径
        key = str(d.resolve())
        if key in seen or not d.is_dir():
            continue
        seen.add(key)
        dirs.append(d)
    return dirs


def find_latest_sample():
    """在常见位置找最新的失败样本原图（排除 *_mask.png）。"""
    newest, newest_mtime = None, -1
    for d in sample_dirs():
        for f in d.glob("*.png"):
            if f.name.endswith("_mask.png"):
                continue
            m = f.stat().st_mtime
            if m > newest_mtime:
                newest, newest_mtime = f, m
    return newest


def list_samples():
    dirs = sample_dirs()
    if not dirs:
        print("没有找到 failure_samples 目录。")
        return 0
    for d in dirs:
        files = sorted(d.glob("*.png"))
        originals = [f for f in files if not f.name.endswith("_mask.png")]
        print(f"{d}  （{len(originals)} 份样本）")
        for f in originals:
            print(f"  {f.name}")
    return 0


def load_image(path):
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        sys.exit(f"读不到图片：{path}")
    if img.ndim == 2:
        sys.exit(f"{path} 是单通道（可能是 mask 文件），请用原图")
    return img


# ---------------------------------------------------------------- 主程序

def run_gui(img, init):
    """带滑动条的交互界面。"""
    win_ctrl = "controls"
    win_view = "preview"
    cv2.namedWindow(win_ctrl, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_ctrl, 420, 360)
    cv2.namedWindow(win_view, cv2.WINDOW_NORMAL)

    def add(name):
        init_val = init[name]
        if name == "min_area":
            init_val = trackbar_from_area(init_val)
        cv2.createTrackbar(name, win_ctrl, int(init_val), TB_MAX[name], lambda v: None)

    for name in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max",
                 "min_area", "kernel_size"]:
        add(name)

    # trackbar 必须挂在一个有图像的窗口上，否则某些平台不显示
    cv2.imshow(win_ctrl, np.zeros((10, 10, 3), np.uint8))

    print("窗口已打开。快捷键：p=打印参数  s=保存  q/ESC=退出")
    last = None

    while True:
        raw = {name: cv2.getTrackbarPos(name, win_ctrl)
               for name in TB_MAX}
        raw["min_area"] = area_from_trackbar(raw["min_area"])  # 映射回真实面积
        # kernel_size 至少要是 1，否则 getStructuringElement 会抛异常
        raw["kernel_size"] = max(1, raw["kernel_size"])
        p = build_params(raw)

        mask = make_mask(img, p)
        dets, total = find_detections(mask, p["min_area"])

        if p != last:  # 参数变了才重画，省点 CPU
            annotated = annotate(img, dets, p, total)
            cv2.imshow(win_view, compose(img, annotated, mask))
            last = p

        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord("q")):
            print_params(p)
            break
        if key == ord("p"):
            print_params(p)
        if key == ord("s"):
            cv2.imwrite("tuned_mask.png", mask)
            cv2.imwrite("tuned_preview.png", compose(img, annotated, mask))
            print("已保存 tuned_mask.png / tuned_preview.png")

    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(
        description="OpenCV 参数调试工具（和 buoy_detector_node 流程一致）")
    ap.add_argument("image", nargs="?", help="要调试的图片；不填则自动找最新失败样本")
    ap.add_argument("-p", "--param", action="append", default=[],
                    metavar="KEY=VALUE", help="初始参数，如 -p min_area=500")
    ap.add_argument("--dump", metavar="MASK.png",
                    help="不打开界面，直接按给定参数处理并写出 mask")
    ap.add_argument("--list", action="store_true", help="列出所有失败样本")
    args = ap.parse_args()

    if args.list:
        return list_samples()

    init = dict(DEFAULTS)
    for item in args.param:
        if "=" not in item:
            sys.exit(f"参数格式应为 KEY=VALUE，收到：{item}")
        k, v = item.split("=", 1)
        k = k.strip()
        if k not in init:
            sys.exit(f"未知参数 {k}，可选：{', '.join(init)}")
        init[k] = int(v)

    if args.image:
        path = Path(args.image).expanduser()
    else:
        path = find_latest_sample()
        if path is None:
            sys.exit("没找到失败样本，请手动指定图片：python3 tune_params.py <图片路径>")
        print(f"自动选择最新样本：{path}")

    img = load_image(path)
    p = build_params(init)

    # ---------- 无界面模式：给脚本/对比用 ----------
    if args.dump:
        mask = make_mask(img, p)
        dets, total = find_detections(mask, p["min_area"])
        cv2.imwrite(args.dump, mask)
        annotated = annotate(img, dets, p, total)
        cv2.imwrite(str(Path(args.dump).with_name(
            Path(args.dump).stem + "_preview.png")), annotated)

        print(f"图片      : {path}  ({img.shape[1]}x{img.shape[0]})")
        print(f"mask      : {args.dump}  (非零像素 {int(np.count_nonzero(mask))})")
        print(f"连通域    : {total} 个，通过 min_area 的 {len(dets)} 个")
        for d in dets[:10]:
            print(f"  面积={d['area']:6d}  bbox={d['bbox']}  score={d['score']:.3f}")
        print_params(p)
        return 0

    # ---------- 交互模式 ----------
    if img.shape[1] > 1400 or img.shape[0] > 900:
        print(f"注意：图片较大（{img.shape[1]}x{img.shape[0]}），已按比例缩放显示，"
              f"但所有计算都在原图上进行。")

    run_gui(img, p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
