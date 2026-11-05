#include "moonwake_camera/failure_recorder.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logging.hpp>

namespace moonwake_camera
{
namespace
{

struct LocalTime
{
  std::tm tm_buf{};
  long msec{0};
};

/// 取本机本地时间 + 毫秒。单独抽出来，是为了让"文件名用到的"和"txt 里用到的"时间只取一次。
LocalTime local_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  LocalTime out;
  localtime_r(&seconds, &out.tm_buf);
  out.msec = static_cast<long>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  return out;
}

/// 2026-11-05 14:32:53.123 +0800
std::string format_wall_clock()
{
  const LocalTime t = local_now();
  char date[64];
  std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &t.tm_buf);
  char zone[16];
  std::strftime(zone, sizeof(zone), "%z", &t.tm_buf);
  char out[128];
  std::snprintf(out, sizeof(out), "%s.%03ld %s", date, t.msec, zone);
  return std::string(out);
}

/// 20261105_143253_123 —— 能按字典序排成时间序
std::string timestamp_for_filename()
{
  const LocalTime t = local_now();
  char date[32];
  std::strftime(date, sizeof(date), "%Y%m%d_%H%M%S", &t.tm_buf);
  char out[64];
  std::snprintf(out, sizeof(out), "%s_%03ld", date, t.msec);
  return std::string(out);
}

/// "STATUS_NO_DETECTION" -> "no_detection"
std::string to_status_slug(std::string name)
{
  if (name.rfind("STATUS_", 0) == 0) {
    name.erase(0, 7);
  }
  std::transform(
    name.begin(), name.end(), name.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return name;
}

std::string size_text(const cv::Mat & mat)
{
  if (mat.empty()) {
    return "n/a";
  }
  return std::to_string(mat.cols) + "x" + std::to_string(mat.rows);
}

}  // namespace

FailureRecorder::FailureRecorder(Options options, rclcpp::Logger logger)
: options_(std::move(options)),
  logger_(std::move(logger)),
  root_(options_.directory)
{
  if (options_.min_interval_sec < 0.0) {
    options_.min_interval_sec = 0.0;
  }

  // 相对路径按启动时的工作目录解析成绝对路径，日志里打出来你才知道文件到底在哪
  std::error_code ec;
  const auto absolute_path = std::filesystem::absolute(root_, ec);
  if (!ec) {
    root_ = absolute_path;
  }

  std::filesystem::create_directories(root_, ec);
  if (ec) {
    RCLCPP_ERROR(
      logger_, "无法创建异常样本目录 '%s': %s",
      root_.c_str(), ec.message().c_str());
  }
}

bool FailureRecorder::should_record()
{
  if (!within_limits()) {
    return false;
  }
  if (options_.min_interval_sec <= 0.0 || !has_saved_) {
    return true;
  }
  // 用 steady_clock 算间隔：ROS 用仿真时间(/clock 回跳)时节流依然有效
  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - last_save_).count();
  return elapsed >= options_.min_interval_sec;
}

bool FailureRecorder::within_limits()
{
  if (!options_.enabled) {
    return false;
  }
  if (options_.max_samples > 0 && saved_ >= options_.max_samples) {
    // 只警告一次，否则每帧都会刷屏
    if (!cap_warned_) {
      cap_warned_ = true;
      RCLCPP_WARN(
        logger_, "异常样本已达上限 %d 份，本次运行不再保存（调大 failure_max_samples 可继续）",
        options_.max_samples);
    }
    return false;
  }
  return true;
}

std::filesystem::path FailureRecorder::make_unique_stem(const std::string & status_slug)
{
  const std::string stamp = timestamp_for_filename();
  const std::string slug = status_slug.empty() ? "abnormal" : status_slug;

  char name[144];
  for (int attempt = 0; attempt < 10000; ++attempt) {
    const int seq = next_seq_ + attempt;
    std::snprintf(name, sizeof(name), "fail_%s_%04d_%s", stamp.c_str(), seq, slug.c_str());
    const std::filesystem::path stem = root_ / name;

    // 时间戳可能因为改系统时间而重复，所以再用"文件是否已存在"兜一层，确保永不覆盖
    std::error_code ec_img;
    std::error_code ec_txt;
    const bool has_image = std::filesystem::exists(stem.string() + ".png", ec_img);
    const bool has_text = std::filesystem::exists(stem.string() + ".txt", ec_txt);
    if (!has_image && !has_text) {
      next_seq_ = seq + 1;
      return stem;
    }
  }

  // 理论上到不了这里：10000 个候选名都被占了就直接用下一个序号
  ++next_seq_;
  std::snprintf(name, sizeof(name), "fail_%s_%04d_%s", stamp.c_str(), next_seq_, slug.c_str());
  return root_ / name;
}

bool FailureRecorder::record(const cv::Mat & image, const cv::Mat & mask, FailureSample sample)
{
  // 这里再判断一次：即使调用方忘了先问 should_record()，也不会绕过节流
  if (!should_record()) {
    ++skipped_;
    return false;
  }

  std::error_code ec;
  std::filesystem::create_directories(root_, ec);
  if (ec) {
    ++failed_;
    RCLCPP_ERROR(
      logger_, "无法创建异常样本目录 '%s': %s", root_.c_str(), ec.message().c_str());
    return false;
  }

  const std::filesystem::path stem = make_unique_stem(to_status_slug(sample.status_name));

  std::filesystem::path image_path = stem;
  image_path += ".png";
  std::filesystem::path mask_path = stem;
  mask_path += "_mask.png";
  std::filesystem::path txt_path = stem;
  txt_path += ".txt";

  std::vector<std::pair<std::string, std::string>> diagnostics = sample.diagnostics;

  // ---- 1. 原图 ----
  bool ok = true;
  std::string image_file = "-";
  if (!image.empty()) {
    if (cv::imwrite(image_path.string(), image)) {
      image_file = image_path.filename().string();
    } else {
      ok = false;
      RCLCPP_ERROR(logger_, "写原图失败: %s", image_path.c_str());
    }
  } else {
    diagnostics.emplace_back("image", "为空(cv_bridge 解码失败)，没有原图可存");
  }

  // ---- 2. mask ----
  // 三种异常里只有 NO_DETECTION 真的跑出过 mask，其余用同尺寸全黑图占位，
  // 这样每个样本都是"原图 + mask + txt"三件套，后续做数据集脚本时不用判断缺文件。
  cv::Mat mask_out;
  if (!mask.empty()) {
    if (mask.type() == CV_8UC1) {
      mask_out = mask;
    } else {
      mask.convertTo(mask_out, CV_8UC1);
    }
  } else if (!image.empty()) {
    mask_out = cv::Mat::zeros(image.size(), CV_8UC1);
    if (sample.mask_source.empty()) {
      sample.mask_source = "placeholder: 本帧没有真实 mask，写入同尺寸全黑图";
    }
  } else {
    sample.mask_source = "n/a(既没有原图也没有 mask)";
  }

  std::string mask_file = "-";
  if (!mask_out.empty()) {
    if (cv::imwrite(mask_path.string(), mask_out)) {
      mask_file = mask_path.filename().string();
    } else {
      ok = false;
      RCLCPP_ERROR(logger_, "写 mask 失败: %s", mask_path.c_str());
    }
  }

  // ---- 3. 同名 txt ----
  // 用一个固定的 "key : value" 行格式：人看得懂，脚本也好解析。
  std::ostringstream os;
  os << std::fixed << std::setprecision(3);
  os << "# moonwake_camera 异常样本记录 (UTF-8)\n";
  os << "# 行格式: key : value ; 以 # 开头是注释, [xxx] 是段落标题\n";
  os << "\n[record]\n";
  os << "saved_at : " << format_wall_clock() << "\n";
  os << "saved_index : " << (saved_ + 1) << "\n";
  os << "status_code : " << static_cast<int>(sample.status_code) << "\n";
  os << "status_name : " << sample.status_name << "\n";
  os << "stage : " << sample.stage << "\n";
  os << "reason : " << sample.reason << "\n";
  os << "header_stamp : " << sample.header_stamp << "\n";
  os << "data_age_ms : " << sample.data_age_ms << "\n";
  os << "process_time_ms : " << sample.process_time_ms << "\n";

  os << "\n[files]\n";
  os << "image_file : " << image_file << "\n";
  os << "mask_file : " << mask_file << "\n";
  os << "mask_source : " << sample.mask_source << "\n";
  os << "image_size : " << size_text(image) << "\n";
  os << "mask_size : " << size_text(mask_out) << "\n";
  os << "mask_nonzero_pixels : " <<
    (mask_out.empty() ? -1 : static_cast<long long>(cv::countNonZero(mask_out))) << "\n";

  os << "\n[params]\n";
  if (sample.params.empty()) {
    os << "(none)\n";
  }
  for (const auto & kv : sample.params) {
    os << kv.first << " : " << kv.second << "\n";
  }

  os << "\n[diagnostics]\n";
  if (diagnostics.empty()) {
    os << "(none)\n";
  }
  for (const auto & kv : diagnostics) {
    os << kv.first << " : " << kv.second << "\n";
  }

  os << "\n[recorder]\n";
  os << "directory : " << root_.string() << "\n";
  os << "min_interval_sec : " << options_.min_interval_sec << "\n";
  os << "max_samples : " << options_.max_samples << "\n";
  os << "saved_before_this : " << saved_ << "\n";
  os << "skipped_before_this : " << skipped_ << "\n";

  std::ofstream ofs(txt_path.string(), std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) {
    ok = false;
    RCLCPP_ERROR(logger_, "打开 txt 失败: %s", txt_path.c_str());
  } else {
    ofs << os.str();
    ofs.flush();
    if (!ofs) {
      ok = false;
      RCLCPP_ERROR(logger_, "写 txt 失败: %s", txt_path.c_str());
    }
  }

  // 无论成功与否都推进节流时间：磁盘写不进去时如果每帧重试，会把日志刷爆
  last_save_ = std::chrono::steady_clock::now();
  has_saved_ = true;

  if (ok) {
    ++saved_;
    RCLCPP_INFO(
      logger_, "已保存异常样本 #%d: %s (status=%s)",
      saved_, stem.filename().c_str(), sample.status_name.c_str());
  } else {
    ++failed_;
    RCLCPP_ERROR(
      logger_, "异常样本未能完整写入: %s", stem.filename().c_str());
  }
  return ok;
}

}  // namespace moonwake_camera
