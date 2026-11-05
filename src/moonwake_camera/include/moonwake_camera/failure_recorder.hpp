#ifndef MOONWAKE_CAMERA__FAILURE_RECORDER_HPP_
#define MOONWAKE_CAMERA__FAILURE_RECORDER_HPP_

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>

namespace moonwake_camera
{

/// 一帧异常样本的"元数据"，会被写进同名 txt。
/// 这里只放数据和字符串，不放 ROS 消息，方便以后单独复用/测试。
struct FailureSample
{
  /// DetectionArray::status 的数值，例如 3 表示 STATUS_INPUT_STALE
  uint8_t status_code{0};
  /// 状态常量名，如 "STATUS_INPUT_STALE"，同时决定文件名后缀
  std::string status_name;
  /// 在哪个检查阶段失败
  std::string stage;
  /// 人类可读的失败原因
  std::string reason;
  /// mask 的来源说明：真实 pipeline 输出 / 占位全黑图
  std::string mask_source;
  /// 图像 header.stamp，格式 "sec.nanosec"
  std::string header_stamp;
  double data_age_ms{0.0};
  double process_time_ms{0.0};
  /// 当时的算法参数快照，按写入顺序保存（不是 map，避免被字母序打乱分组）
  std::vector<std::pair<std::string, std::string>> params;
  /// 额外诊断信息（连通域数量、mask 像素数等）
  std::vector<std::pair<std::string, std::string>> diagnostics;
};

class FailureRecorder
{
public:
  struct Options
  {
    /// 保存目录，相对路径按节点启动时的工作目录解析
    std::string directory{"failure_samples"};
    /// 两次落盘之间至少间隔多少秒，0 表示不限速
    double min_interval_sec{10.0};
    /// 单次运行最多保存多少份，<=0 表示不限制（防止把磁盘写满）
    int max_samples{200};
    bool enabled{true};
  };

  FailureRecorder(Options options, rclcpp::Logger logger);

  /// 只做判断、不产生副作用：现在是否允许再存一份。
  /// 节点可以先问它，再决定要不要做解码/mask 计算这类昂贵操作。
  /// （唯一副作用：样本数达到上限时打一条警告，避免静默停止保存）
  bool should_record();

  /// 真正落盘：原图 png + mask png + 同名 txt。
  /// 返回 true 表示三个文件都写成功。
  bool record(const cv::Mat & image, const cv::Mat & mask, FailureSample sample);

  int saved_count() const {return saved_;}
  int skipped_count() const {return skipped_;}
  int failed_count() const {return failed_;}
  const std::filesystem::path & root() const {return root_;}

private:
  /// 带"到达上限只警告一次"的检查
  bool within_limits();
  /// 生成不与已有文件冲突的文件名主干（不含扩展名、含目录）
  std::filesystem::path make_unique_stem(const std::string & status_slug);

  Options options_;
  rclcpp::Logger logger_;
  std::filesystem::path root_;
  /// 用 steady_clock 算间隔：即使 ROS 用仿真时间(/clock 跳变)节流也不会失效
  std::chrono::steady_clock::time_point last_save_{};
  bool has_saved_{false};
  bool cap_warned_{false};
  int saved_{0};
  int skipped_{0};
  int failed_{0};
  int next_seq_{1};
};

}  // namespace moonwake_camera

#endif  // MOONWAKE_CAMERA__FAILURE_RECORDER_HPP_
