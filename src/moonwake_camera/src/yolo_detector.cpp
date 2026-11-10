#include "moonwake_camera/yolo_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace moonwake_camera
{
namespace
{

constexpr int kPadValue = 114;   // letterbox 补边灰度值（模型契约）

std::vector<std::string> split_class_names(const std::string & text)
{
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto first = item.find_first_not_of(" \t");
    const auto last = item.find_last_not_of(" \t");
    if (first == std::string::npos) {continue;}
    out.push_back(item.substr(first, last - first + 1));
  }
  return out;
}

/// 等比缩放到 size×size 并居中补灰边；
/// 同时输出 scale 和 padding，供后面把框还原到原图坐标
cv::Mat letterbox(const cv::Mat & bgr, int size, float & scale, int & pad_x, int & pad_y)
{
  scale = std::min(
    size / static_cast<float>(bgr.cols),
    size / static_cast<float>(bgr.rows));

  const int new_w = std::min(size, static_cast<int>(std::round(bgr.cols * scale)));
  const int new_h = std::min(size, static_cast<int>(std::round(bgr.rows * scale)));
  pad_x = (size - new_w) / 2;
  pad_y = (size - new_h) / 2;

  cv::Mat canvas(size, size, CV_8UC3, cv::Scalar(kPadValue, kPadValue, kPadValue));
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));
  return canvas;
}

}  // namespace

struct YoloDetector::Impl
{
#ifdef HAVE_ONNXRUNTIME
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "moonwake_camera"};
  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::unique_ptr<Ort::Session> session;
  std::string input_name;
  std::string output_name;
#endif
};

YoloDetector::YoloDetector(rclcpp::Node & node)
: impl_(std::make_unique<Impl>())
{
  //------------ 读参数 ------------
  model_path_ = node.get_parameter("model_path").as_string();
  conf_threshold_ = node.get_parameter("model_conf_threshold").as_double();
  iou_threshold_ = node.get_parameter("model_iou_threshold").as_double();
  input_size_ = static_cast<int>(node.get_parameter("model_input_size").as_int());
  num_threads_ = static_cast<int>(node.get_parameter("model_num_threads").as_int());
  class_names_ = split_class_names(node.get_parameter("model_class_names").as_string());

  //------------ 配置检查：权重/参数不一致 -> CONFIG_ERROR ------------
  if (model_path_.empty()) {
    config_error_ = "model_path 为空：选 yolo 后端时必须给 -p model_path:=/path/model.onnx";
    return;
  }
  if (!std::filesystem::is_regular_file(model_path_)) {
    config_error_ = "模型文件不存在: " + model_path_;
    return;
  }
  if (class_names_.empty()) {
    config_error_ = "model_class_names 为空";
    return;
  }
  if (input_size_ <= 0 || input_size_ % 32 != 0) {
    config_error_ = "model_input_size 必须是 32 的正整数倍，当前为 " +
      std::to_string(input_size_);
    return;
  }
  if (conf_threshold_ <= 0.0 || conf_threshold_ > 1.0) {
    config_error_ = "model_conf_threshold 必须在 (0,1] 之间，当前为 " +
      std::to_string(conf_threshold_);
    return;
  }
  if (iou_threshold_ <= 0.0 || iou_threshold_ > 1.0) {
    config_error_ = "model_iou_threshold 必须在 (0,1] 之间，当前为 " +
      std::to_string(iou_threshold_);
    return;
  }

#ifdef HAVE_ONNXRUNTIME
  //------------ 加载模型：失败 -> MODEL_ERROR ------------
  try {
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    //默认交给 ORT 决定；若显式指定则用它（核多的机器上限制线程数往往更快）
    if (num_threads_ > 0) {
      opts.SetIntraOpNumThreads(num_threads_);
    }

    impl_->session = std::make_unique<Ort::Session>(impl_->env, model_path_.c_str(), opts);

    Ort::AllocatorWithDefaultOptions alloc;
    impl_->input_name = impl_->session->GetInputNameAllocated(0, alloc).get();
    impl_->output_name = impl_->session->GetOutputNameAllocated(0, alloc).get();

    //模型输入尺寸要和 model_input_size 对得上，否则 reszie 出来的东西喂不进去
    const auto in_shape =
      impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() == 4 && in_shape[2] > 0 && in_shape[2] != input_size_) {
      config_error_ = "model_input_size=" + std::to_string(input_size_) +
        " 与模型实际输入 " + std::to_string(in_shape[2]) + " 不一致";
    }
  } catch (const std::exception & e) {
    impl_->session.reset();
    model_error_ = std::string("加载模型失败: ") + e.what();
  }
#else
  model_error_ =
    "编译时未找到 ONNX Runtime，YOLO 后端不可用。"
    "安装后重新构建：cmake -DONNXRUNTIME_ROOT=/path/to/onnxruntime";
#endif
}

YoloDetector::~YoloDetector() = default;

std::vector<IDetector::Detection> YoloDetector::detect(const cv::Mat & bgr)
{
  //构造期发现的问题，按类型抛出去由节点映射成状态码
  if (!config_error_.empty()) {
    throw DetectorConfigError(config_error_);
  }
  if (!model_error_.empty()) {
    throw DetectorModelError(model_error_);
  }
  if (bgr.empty()) {
    return {};
  }

#ifdef HAVE_ONNXRUNTIME
  //------------ 1. letterbox ------------
  float scale = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
  const cv::Mat padded = letterbox(bgr, input_size_, scale, pad_x, pad_y);

  //------------ 2. blob（scale=1/255, swapRB=true）------------
  // 这里不能加 const：Ort::Value::CreateTensor 要的是 float*（指针只读，但签名如此）
  cv::Mat blob = cv::dnn::blobFromImage(
    padded, 1.0 / 255.0, cv::Size(input_size_, input_size_), cv::Scalar(), true, false);

  //------------ 3. 推理 ------------
  std::array<int64_t, 4> input_shape{1, 3, input_size_, input_size_};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    impl_->mem_info, blob.ptr<float>(), blob.total(), input_shape.data(), input_shape.size());

  const char * input_names[] = {impl_->input_name.c_str()};
  const char * output_names[] = {impl_->output_name.c_str()};

  std::vector<Ort::Value> outputs;
  try {
    outputs = impl_->session->Run(
      Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
  } catch (const std::exception & e) {
    throw DetectorModelError(std::string("forward 失败: ") + e.what());
  }

  //------------ 4. 输出布局 [1, 4+nc, n] ------------
  const auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  if (out_shape.size() != 3) {
    throw DetectorModelError("模型输出维度不是 3，无法解析");
  }
  const int channels = static_cast<int>(out_shape[1]);
  const int num_candidates = static_cast<int>(out_shape[2]);
  const int num_classes = channels - 4;
  if (num_classes <= 0 || num_candidates <= 0) {
    throw DetectorModelError("模型输出形状异常: " + std::to_string(channels) + "x" +
            std::to_string(num_candidates));
  }

  // 数据按 [batch][channel][candidate] 排布，取值就是 data[channel * n + i]
  const float * data = outputs[0].GetTensorData<float>();

  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int i = 0; i < num_candidates; ++i) {
    float best_score = 0.0f;
    int best_class = -1;
    for (int c = 0; c < num_classes; ++c) {
      const float s = data[(4 + c) * num_candidates + i];
      if (s > best_score) {
        best_score = s;
        best_class = c;
      }
    }
    if (best_score < conf_threshold_) {continue;}

    //YOLO 的 4 个框值是 xywh（中心点 + 宽高），单位是 640 输入图的像素
    const float cx = data[0 * num_candidates + i];
    const float cy = data[1 * num_candidates + i];
    const float w = data[2 * num_candidates + i];
    const float h = data[3 * num_candidates + i];

    boxes.emplace_back(cx - w * 0.5, cy - h * 0.5, w, h);
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  //------------ 5. NMS ------------
  std::vector<int> keep;
  if (!boxes.empty()) {
    cv::dnn::NMSBoxes(boxes, scores,
      static_cast<float>(conf_threshold_), static_cast<float>(iou_threshold_), keep);
  }

  //------------ 6. 坐标还原：先减 pad，再除 scale ------------
  std::vector<Detection> detections;
  detections.reserve(keep.size());
  for (const int idx : keep) {
    const cv::Rect2d & r = boxes[idx];

    float x1 = (static_cast<float>(r.x) - pad_x) / scale;
    float y1 = (static_cast<float>(r.y) - pad_y) / scale;
    float x2 = (static_cast<float>(r.x + r.width) - pad_x) / scale;
    float y2 = (static_cast<float>(r.y + r.height) - pad_y) / scale;

    //裁到原图范围内（和 ultralytics 的 clip_boxes 行为一致）
    x1 = std::clamp(x1, 0.0f, static_cast<float>(bgr.cols));
    y1 = std::clamp(y1, 0.0f, static_cast<float>(bgr.rows));
    x2 = std::clamp(x2, 0.0f, static_cast<float>(bgr.cols));
    y2 = std::clamp(y2, 0.0f, static_cast<float>(bgr.rows));

    Detection det;
    const int cid = class_ids[idx];
    det.class_name = (cid >= 0 && cid < static_cast<int>(class_names_.size()))
      ? class_names_[cid]
      : ("class_" + std::to_string(cid));
    det.center.x = (x1 + x2) * 0.5;
    det.center.y = (y1 + y2) * 0.5;
    det.center.z = 0.0;
    det.bbox = {x1, y1, x2, y2};
    det.score = scores[idx];

    detections.push_back(det);
  }

  //没检出时写进失败样本的原因
  last_reason_ = "推理正常，但没有置信度 >= model_conf_threshold=" +
    std::to_string(conf_threshold_) + " 的目标";

  return detections;
#else
  throw DetectorModelError(model_error_);
#endif
}

}  // namespace moonwake_camera
