#include "moonwake_camera/hsv_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <string>

namespace moonwake_camera
{

HsvDetector::HsvDetector(rclcpp::Node & node)
: node_(node)
{
}

std::vector<IDetector::Detection> HsvDetector::detect(const cv::Mat & bgr)
{
  //------------ 参数检查（原 sync_callback 里的 "检查配置有效性"）------------
  const int h_min = node_.get_parameter("h_min").as_int();
  const int h_max = node_.get_parameter("h_max").as_int();
  const int s_min = node_.get_parameter("s_min").as_int();
  const int s_max = node_.get_parameter("s_max").as_int();
  const int v_min = node_.get_parameter("v_min").as_int();
  const int v_max = node_.get_parameter("v_max").as_int();
  const int min_area = node_.get_parameter("min_area").as_int();
  const int kernel_size = node_.get_parameter("kernel_size").as_int();

  if (h_min >= h_max || s_min >= s_max || v_min >= v_max || min_area <= 0 || kernel_size <= 0) {
    //把到底哪一项不合法写清楚，方便事后看 txt 定位
    std::string why;
    if (h_min >= h_max) {why += "h_min>=h_max; ";}
    if (s_min >= s_max) {why += "s_min>=s_max; ";}
    if (v_min >= v_max) {why += "v_min>=v_max; ";}
    if (min_area <= 0) {why += "min_area<=0; ";}
    if (kernel_size <= 0) {why += "kernel_size<=0; ";}
    if (why.size() >= 2) {why.erase(why.size() - 2);}   //去掉结尾多出来的 "; "
    throw DetectorConfigError(why);
  }

  //------------ 第一重过滤：全局阈值 ------------
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat color_mask;
  cv::Scalar lower(h_min, s_min, v_min);
  cv::Scalar upper(h_max, s_max, v_max);
  cv::inRange(hsv, lower, upper, color_mask);

  //------------ 第二重过滤：自适应阈值 ------------
  cv::Mat adaptive_mask;
  int block_size = node_.get_parameter("adaptive_block_size").as_int();
  const double adaptive_C = node_.get_parameter("adaptive_C").as_double();
  if (block_size % 2 == 0) {
    block_size += 1;
  }
  cv::adaptiveThreshold(color_mask, adaptive_mask, 255,
    cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, block_size, adaptive_C);

  cv::Mat mask;
  cv::bitwise_and(color_mask, adaptive_mask, mask);

  //------------ 形态学：先开运算去噪，再闭运算填洞 ------------
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
    cv::Size(kernel_size, kernel_size));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  //留给 debug_mask 话题用
  last_mask_ = mask;

  //------------ 连通域分析 ------------
  cv::Mat labels, stats, centroids;
  const int n_labels = cv::connectedComponentsWithStats(mask, labels, stats, centroids);

  //------------ 筛选候选 ------------
  std::vector<Detection> candidates;
  for (int i = 1; i < n_labels; ++i) {
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < min_area) {continue;}

    Detection det;
    det.class_name = "buoy";
    det.center.x = centroids.at<double>(i, 0);
    det.center.y = centroids.at<double>(i, 1);
    det.center.z = 0.0;

    const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(i, cv::CC_STAT_TOP);
    const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    det.bbox = {
      static_cast<float>(x),
      static_cast<float>(y),
      static_cast<float>(w + x),
      static_cast<float>(h + y)
    };

    const float bbox_area = static_cast<float>(w * h);
    det.score = (bbox_area > 0) ? static_cast<float>(area) / bbox_area : 0.0f;
    candidates.push_back(det);
  }

  //没检出时写进失败样本的原因
  last_reason_ = "mask 里有 " + std::to_string(n_labels - 1) +
    " 个连通域，全部小于 min_area=" + std::to_string(min_area);

  return candidates;
}

}  // namespace moonwake_camera
