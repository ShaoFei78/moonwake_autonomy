#include "moonwake_camera/detector.hpp"

#include <memory>
#include <string>

#include "moonwake_camera/hsv_detector.hpp"
#include "moonwake_camera/yolo_detector.hpp"

namespace moonwake_camera
{

std::unique_ptr<IDetector> create_detector(rclcpp::Node & node)
{
  const std::string backend = node.get_parameter("detector_backend").as_string();

  if (backend == "hsv") {
    return std::make_unique<HsvDetector>(node);
  }
  if (backend == "yolo") {
    return std::make_unique<YoloDetector>(node);
  }

  // 写错后端名时不要静默，也不要让节点崩掉（崩了话题就全没了，很难查）。
  // 这里明确报错并回退到 hsv，节点照常启动。
  RCLCPP_ERROR(
    node.get_logger(),
    "未知的 detector_backend='%s'（可选 hsv / yolo），已回退到 hsv",
    backend.c_str());
  return std::make_unique<HsvDetector>(node);
}

}  // namespace moonwake_camera
