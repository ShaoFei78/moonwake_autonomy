#ifndef MOONWAKE_CAMERA__YOLO_DETECTOR_HPP_
#define MOONWAKE_CAMERA__YOLO_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "moonwake_camera/detector.hpp"

namespace moonwake_camera
{

/// 基于 ONNX Runtime 的 YOLO 检测后端。
///
/// 推理流程（与 perception_release_v1/preprocessing.md 的契约一致）：
///   BGR → letterbox 到 640×640（等比缩放，灰边 114）
///       → blobFromImage(scale=1/255, swapRB=true)
///       → session.Run
///       → [1, 4+nc, 8400] 转置成 [8400, 4+nc]
///       → conf 过滤 + NMSBoxes(iou)
///       → 坐标还原（先减 pad，再除 scale）
class YoloDetector : public IDetector
{
public:
  explicit YoloDetector(rclcpp::Node & node);
  ~YoloDetector() override;

  std::vector<Detection> detect(const cv::Mat & bgr) override;

  const char * name() const override {return "yolo";}

  std::string last_reason() const override {return last_reason_;}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  std::string model_path_;
  double conf_threshold_{0.25};
  double iou_threshold_{0.7};
  int input_size_{640};
  int num_threads_{0};
  std::vector<std::string> class_names_;

  /// 非空表示构造期就发现了问题，按类型在 detect() 里抛出去
  std::string config_error_;
  std::string model_error_;

  /// 最近一帧"为什么没检出"的说明，写进失败样本
  std::string last_reason_;
};

}  // namespace moonwake_camera

#endif  // MOONWAKE_CAMERA__YOLO_DETECTOR_HPP_
