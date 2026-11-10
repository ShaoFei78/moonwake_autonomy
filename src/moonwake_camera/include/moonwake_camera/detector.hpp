#ifndef MOONWAKE_CAMERA__DETECTOR_HPP_
#define MOONWAKE_CAMERA__DETECTOR_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "moonwake_camera/msg/detection.hpp"

namespace moonwake_camera
{

/// 检测后端的统一接口。
///
/// 约定（两个后端必须完全一致）：
///   * 输入是一张 BGR 八位三通道图，尺寸任意
///   * 输出的 Detection 用**原图像素坐标**：
///       class_name 类别名
///       center     (x, y) 像素，z = 0
///       bbox       [x_min, y_min, x_max, y_max]
///       score      0~1 的置信度
///   * 没有目标是正常情况，返回空 vector（节点据此填 STATUS_NO_DETECTION）
///   * 参数非法 -> 抛 DetectorConfigError（节点填 STATUS_CONFIG_ERROR）
///   * 模型加载失败 / 推理失败 -> 抛 DetectorModelError（节点填 STATUS_MODEL_ERROR）
class IDetector
{
public:
  using Detection = moonwake_camera::msg::Detection;

  virtual ~IDetector() = default;

  /// 对一帧做检测。坐标必须是原图像素坐标。
  virtual std::vector<Detection> detect(const cv::Mat & bgr) = 0;

  /// 后端名字，用于启动日志
  virtual const char * name() const = 0;

  /// 可选：本帧的可视化 mask（只有 HSV 后端有，YOLO 返回空）
  virtual cv::Mat debug_mask() const {return cv::Mat();}

  /// 可选：本帧"为什么没检出"的可读说明，写进失败样本的 txt
  virtual std::string last_reason() const {return std::string();}
};

/// 参数/配置层面的错误（例如 HSV 阈值上下界颠倒、模型路径为空）
class DetectorConfigError : public std::runtime_error
{
public:
  explicit DetectorConfigError(const std::string & what) :std::runtime_error(what) {}
};

/// 模型层面的错误（加载失败、forward 抛异常）
class DetectorModelError : public std::runtime_error
{
public:
  explicit DetectorModelError(const std::string & what) :std::runtime_error(what) {}
};

/// 按参数 detector_backend（hsv / yolo）创建后端。
/// 参数值无法识别时回退到 hsv，并在日志里报错（保证节点仍能启动、话题仍然存在）。
std::unique_ptr<IDetector> create_detector(rclcpp::Node & node);

}  // namespace moonwake_camera

#endif  // MOONWAKE_CAMERA__DETECTOR_HPP_
