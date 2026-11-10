#ifndef MOONWAKE_CAMERA__HSV_DETECTOR_HPP_
#define MOONWAKE_CAMERA__HSV_DETECTOR_HPP_

#include <string>
#include <vector>

#include "moonwake_camera/detector.hpp"

namespace moonwake_camera
{

/// 原来的 HSV 阈值 + 自适应阈值 + 连通域方案，逻辑原样搬过来。
/// 参数每帧现读，所以 ros2 param set 改完下一帧就生效（和重构前一致）。
class HsvDetector : public IDetector
{
public:
  explicit HsvDetector(rclcpp::Node & node);

  std::vector<Detection> detect(const cv::Mat & bgr) override;

  const char * name() const override {return "hsv";}

  /// 最近一帧的二值 mask，供 debug_mask 话题使用
  cv::Mat debug_mask() const override {return last_mask_;}

  std::string last_reason() const override {return last_reason_;}

private:
  rclcpp::Node & node_;
  cv::Mat last_mask_;
  std::string last_reason_;
};

}  // namespace moonwake_camera

#endif  // MOONWAKE_CAMERA__HSV_DETECTOR_HPP_
