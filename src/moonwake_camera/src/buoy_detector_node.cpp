#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "moonwake_camera/msg/detection_array.hpp"
#include "moonwake_camera/detector.hpp"
#include "moonwake_camera/failure_recorder.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "sensor_msgs/msg/camera_info.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace
{
//图像 header.stamp 打成 "sec.nanosec"，写进异常样本的 txt
std::string format_stamp(const builtin_interfaces::msg::Time & stamp)
{
    char buf[48];
    std::snprintf(buf,sizeof(buf),"%d.%09u",stamp.sec,stamp.nanosec);
    return std::string(buf);
}
//double 转成固定小数的字符串，避免 to_string 输出一长串 0
std::string format_double(double value,int precision=3)
{
    char buf[64];
    std::snprintf(buf,sizeof(buf),"%.*f",precision,value);
    return std::string(buf);
}
//默认的异常样本目录：<本包目录>/failure_samples
//__FILE__ 在编译时被替换成源文件的绝对路径，例如
//  /home/shaofei/moonwake_autonomy/src/moonwake_camera/src/buoy_detector_node.cpp
//往上两级就是包目录 src/moonwake_camera。
//这样样本会稳定落在包里，不受"从哪个目录启动节点"影响。
std::string default_failure_dir()
{
    const std::filesystem::path source_file(__FILE__);
    if (!source_file.is_absolute()) {
        //少数构建方式会传相对路径，此时推导不可靠，退化成相对当前目录
        return "failure_samples";
    }
    return (source_file.parent_path().parent_path() / "failure_samples").string();
}
}  // namespace

// =====================================================================
//  跟踪（卡尔曼滤波 + 数据关联）
// ---------------------------------------------------------------------
//  这一段解决什么问题：
//    检测器每帧只会说"这儿有个框"，它不知道"这是不是上一帧那个"。
//    跟踪器补上"身份"，下游才能说"1 号浮标往左了"，
//    而不是"又冒出来一个浮标"。
//
//  三个部件：
//    ① 卡尔曼滤波：记住"它在哪、往哪走"，并能预测下一帧它在哪
//    ② 数据关联  ：判断"这一帧的框，是不是某条轨迹的"
//    ③ 状态机    ：连续见到几次才算真目标；多久没见到就丢弃
// =====================================================================
namespace
{

// ---------- 卡尔曼的状态向量（8 维，顺序固定） ----------
//   [0] cx   框中心 x（像素）
//   [1] cy   框中心 y（像素）
//   [2] w    框宽（像素）
//   [3] h    框高（像素）
//   [4] vx   中心 x 每秒动多少像素
//   [5] vy   中心 y 每秒动多少像素
//   [6] vw   框每秒变宽/变窄多少
//   [7] vh   框每秒变高/变矮多少
//
// 观测量只有 4 维：[cx, cy, w, h]。
// 速度从来没有被"直接测量"过，只能靠"位置随时间的变化"反推出来。
constexpr int kStateDim = 8;
constexpr int kMeasDim = 4;

// ---------------- 一条轨迹 = 一个"档案袋" ----------------
struct KalmanTrack
{
    int id = 0;                 // 编号，给下游认人用
    int hits = 0;               // 连续"见到"的次数
    int misses = 0;             // 连续"没见到"的次数
    bool confirmed = false;     // 见过够多次才算确认（防误检）
    rclcpp::Time last_stamp;    // 上一帧的时间戳，用来算 dt
    cv::KalmanFilter kf{kStateDim, kMeasDim, 0, CV_32F};

    KalmanTrack(int track_id, const cv::Rect2f & box, const rclcpp::Time & stamp)
    : id(track_id), last_stamp(stamp)
    {
        // ---- F：状态转移矩阵。一句话："位置 += dt × 速度，速度不变" ----
        // 先铺单位阵（表示"每个量等于它自己"），再把 4 个 dt 填进去。
        // 注意：这里的 1.0f 只是占位，真正的 dt 每帧在 predict() 里重填。
        kf.transitionMatrix = cv::Mat::eye(kStateDim, kStateDim, CV_32F);
        kf.transitionMatrix.at<float>(0, 4) = 1.0f;   // cx = 1*cx + dt*vx
        kf.transitionMatrix.at<float>(1, 5) = 1.0f;   // cy = 1*cy + dt*vy
        kf.transitionMatrix.at<float>(2, 6) = 1.0f;   // w  = 1*w  + dt*vw
        kf.transitionMatrix.at<float>(3, 7) = 1.0f;   // h  = 1*h  + dt*vh

        // ---- H：观测矩阵。它的作用就是"只取前 4 维" ----
        // 意思是：我测得到位置和大小，测不到速度。
        kf.measurementMatrix = cv::Mat::zeros(kMeasDim, kStateDim, CV_32F);
        for (int i = 0; i < kMeasDim; ++i) {
            kf.measurementMatrix.at<float>(i, i) = 1.0f;
        }

        // ---- Q：过程噪声。"我有多不相信匀速这个假设" ----
        // 速度那一半给得比位置大：船在摇，目标的真实速度随时会变。
        kf.processNoiseCov = cv::Mat::eye(kStateDim, kStateDim, CV_32F) * 1e-2f;
        for (int i = kMeasDim; i < kStateDim; ++i) {
            kf.processNoiseCov.at<float>(i, i) = 1e-1f;
        }

        // ---- R：测量噪声。"检测框本身有多抖" ----
        // 25 相当于承认"框中心每帧会抖 ±5 像素"。
        // 检测框越不稳，这个值应该越大，滤波器就会更依赖预测。
        kf.measurementNoiseCov = cv::Mat::eye(kMeasDim, kMeasDim, CV_32F) * 25.0f;

        // ---- P0：初始不确定度 ----
        // 位置是刚测到的，比较确定；速度完全不知道，给一个很大的数。
        kf.errorCovPost = cv::Mat::eye(kStateDim, kStateDim, CV_32F) * 10.0f;
        for (int i = kMeasDim; i < kStateDim; ++i) {
            kf.errorCovPost.at<float>(i, i) = 1000.0f;
        }

        // ---- 初始状态：用第一帧检测到的框 ----
        kf.statePost = cv::Mat::zeros(kStateDim, 1, CV_32F);
        kf.statePost.at<float>(0) = box.x + box.width * 0.5f;
        kf.statePost.at<float>(1) = box.y + box.height * 0.5f;
        kf.statePost.at<float>(2) = box.width;
        kf.statePost.at<float>(3) = box.height;
        // 速度 4~7 维保持 0：刚出现，还不知道它往哪走
        hits = 1;
    }

    // 把 8 维状态向量转回"左上角 + 宽高"的框
    static cv::Rect2f to_rect(const cv::Mat & s)
    {
        const float cx = s.at<float>(0);
        const float cy = s.at<float>(1);
        // 极端情况下滤波器可能算出负数宽高，夹一下，避免出现"反向框"
        const float w = std::max(1.0f, s.at<float>(2));
        const float h = std::max(1.0f, s.at<float>(3));
        return cv::Rect2f(cx - w * 0.5f, cy - h * 0.5f, w, h);
    }

    // 每帧开始调一次：预测"这一帧它会在哪"
    cv::Rect2f predict(const rclcpp::Time & stamp)
    {
        float dt = static_cast<float>((stamp - last_stamp).seconds());
        // 时间戳异常（重复、回跳、暂停太久）时按 30fps 兜底，
        // 否则 dt 会变成 0 或几百秒，预测彻底失真
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 1.0f / 30.0f;
        }
        last_stamp = stamp;

        // 把这一帧真正的 dt 填进 F
        kf.transitionMatrix.at<float>(0, 4) = dt;
        kf.transitionMatrix.at<float>(1, 5) = dt;
        kf.transitionMatrix.at<float>(2, 6) = dt;
        kf.transitionMatrix.at<float>(3, 7) = dt;

        kf.predict();

        // ★ 关键一步：把"预测"提拔成"当前状态"
        //   OpenCV 的 predict() 只更新 statePre / errorCovPre，不动 statePost；
        //   而下一帧的 predict() 又是从 statePost 出发的。
        //   不搬这一次的话，连续漏检时预测会原地打转、不往前走。
        kf.statePre.copyTo(kf.statePost);
        kf.errorCovPre.copyTo(kf.errorCovPost);

        return to_rect(kf.statePre);
    }

    // 关联成功后调一次：用检测框修正状态
    void update(const cv::Rect2f & box, int min_hits)
    {
        cv::Mat z(kMeasDim, 1, CV_32F);
        z.at<float>(0) = box.x + box.width * 0.5f;
        z.at<float>(1) = box.y + box.height * 0.5f;
        z.at<float>(2) = box.width;
        z.at<float>(3) = box.height;

        // correct() 内部算的就是那套：新息 -> 增益 -> 修正状态和协方差
        kf.correct(z);

        ++hits;
        misses = 0;
        if (hits >= min_hits) {
            confirmed = true;   // 连续见到够多次，才认为它是真目标
        }
    }

    void mark_missed() {++misses;}

    // 修正后的框（关联成功之后才有意义）
    cv::Rect2f box() const {return to_rect(kf.statePost);}
};

// ---------------- 两个框重叠了多少 ----------------
// IoU = 交集面积 / 并集面积
//   1.0 = 完全重合；0.0 = 完全不挨着
float box_iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.width, b.x + b.width);
    const float y2 = std::min(a.y + a.height, b.y + b.height);

    const float w = x2 - x1;
    const float h = y2 - y1;
    if (w <= 0.0f || h <= 0.0f) {return 0.0f;}   // 不相交

    const float inter = w * h;
    const float uni = a.width * a.height + b.width * b.height - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

}  // namespace

class BuoyDetectorNode : public rclcpp::Node
{
    public:
        using DetectionArray = moonwake_camera::msg::DetectionArray;
        using Detection = moonwake_camera::msg::Detection;
        using FailureSample = moonwake_camera::FailureSample;
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::CameraInfo>;

        BuoyDetectorNode() : Node("buoy_detector_node")
        {
          //声明参数
          this->declare_parameter<int>("h_min", 35);
          this->declare_parameter<int>("h_max", 85);
          this->declare_parameter<int>("s_min", 75);
          this->declare_parameter<int>("s_max", 255);
          this->declare_parameter<int>("v_min", 41);
          this->declare_parameter<int>("v_max", 255);
          this->declare_parameter<int>("min_area", 100);
          this->declare_parameter<int>("kernel_size", 8);
          this->declare_parameter<double>("stale_threshold_sec", 0.5);
          this->declare_parameter<bool>("publish_debug_mask", false);
          //标注图开关：把检测框画在原图上，发布到 debug 话题（默认开启）
          this->declare_parameter<bool>("publish_debug_image", true);
          this->declare_parameter<int>("adaptive_block_size",11);
          this->declare_parameter<double>("adaptive_C",2.0);
          //异常样本保存相关参数
          this->declare_parameter<bool>("save_failure_samples", true);
          //留空 = 自动存到 <本包目录>/failure_samples/（见 default_failure_dir）
          //填了路径就用你给的路径，相对路径相对"启动节点时的当前目录"
          this->declare_parameter<std::string>("failure_dir", "");
          this->declare_parameter<double>("failure_min_interval_sec", 10.0);
          this->declare_parameter<int>("failure_max_samples", 200);
          //检测后端选择：hsv（原来的 OpenCV 方案）或 yolo（ONNX Runtime）
          this->declare_parameter<std::string>("detector_backend","hsv");
          //yolo 后端专用参数
          this->declare_parameter<std::string>("model_path","");
          this->declare_parameter<double>("model_conf_threshold",0.25);
          this->declare_parameter<double>("model_iou_threshold",0.7);
          this->declare_parameter<int>("model_input_size",640);
          //推理线程数：0=交给 ONNX Runtime 自己决定；实测 4 线程往往比默认更快
          this->declare_parameter<int>("model_num_threads",0);
          //类别名，多个用逗号分隔；单类模型就是 "green_buoy"
          this->declare_parameter<std::string>("model_class_names","green_buoy");

          //---------------- 跟踪参数 ----------------
          //总开关：关掉之后行为和以前完全一样（Detection.track_id 恒为 -1）
          this->declare_parameter<bool>("track_enable",true);
          //连续见到几次才算"确认"。调大=更保守，调小=响应更快但可能被误检骗
          this->declare_parameter<int>("track_min_hits",3);
          //连续丢失多少帧后判为"丢失"（这之后就不再画它的预测框）
          this->declare_parameter<int>("track_max_age",15);
          //连续丢失多少帧后直接删除这条轨迹（30fps 下 45 帧约 1.5 秒）
          this->declare_parameter<int>("track_max_lost",45);
          //门限一：预测框和检测框的 IoU 低于这个值，就不许配对
          this->declare_parameter<double>("track_iou_gate",0.10);
          //门限二：两个框中心距离超过这个像素数，也不许配对
          this->declare_parameter<double>("track_dist_gate",200.0);

          //创建发布者
          result_pub_=this->create_publisher<DetectionArray>("detection",10);
          debug_pub_=this->create_publisher<sensor_msgs::msg::Image>("debug_mask",10);
          //标注图：检测框画在原图上，方便直接用 rqt_image_view 看效果
          debug_image_pub_=this->create_publisher<sensor_msgs::msg::Image>("debug",10);
          //创建订阅者
          image_sub_.subscribe(this,"image_raw");
          info_sub_.subscribe(this,"camera_info");
          sync_=std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),image_sub_,info_sub_);
          sync_->registerCallback(std::bind(&BuoyDetectorNode::sync_callback,this,std::placeholders::_1,std::placeholders::_2));
          //创建异常样本记录器
          moonwake_camera::FailureRecorder::Options rec_opt;
          rec_opt.enabled=this->get_parameter("save_failure_samples").as_bool();
          rec_opt.directory=this->get_parameter("failure_dir").as_string();
          if(rec_opt.directory.empty()){
              //没指定就落到 <本包目录>/failure_samples/
              rec_opt.directory=default_failure_dir();
          }
          rec_opt.min_interval_sec=this->get_parameter("failure_min_interval_sec").as_double();
          rec_opt.max_samples=this->get_parameter("failure_max_samples").as_int();
          failure_recorder_=std::make_unique<moonwake_camera::FailureRecorder>(rec_opt,this->get_logger());
          RCLCPP_INFO(this->get_logger(),"failure samples -> %s (%s, min interval %.1f s, max %d)",
                      failure_recorder_->root().c_str(),
                      rec_opt.enabled?"enabled":"disabled",
                      rec_opt.min_interval_sec,rec_opt.max_samples);
          //启动时打印自适应阈值参数
          //as_int() 返回的是 int64_t，直接塞进 %d 类型不匹配（会有 -Wformat 警告），
          //所以先存成 int 再打印（block_size 本来就只有几十，不会溢出）
          int block_size=this->get_parameter("adaptive_block_size").as_int();
          double adaptive_c=this->get_parameter("adaptive_C").as_double();
          RCLCPP_INFO(this->get_logger(),"adaptive params: block_size=%d, C=%.1f",
                      block_size,adaptive_c);
          //创建检测后端（hsv / yolo），之后的检测一律走 detector_->detect()
          detector_=moonwake_camera::create_detector(*this);
          RCLCPP_INFO(this->get_logger(),"detector backend: %s",detector_->name());

          //---------------- 读取跟踪参数 ----------------
          //这几个参数只在启动时读一次；改完要重启节点才生效
          track_enable_   = this->get_parameter("track_enable").as_bool();
          track_min_hits_ = static_cast<int>(this->get_parameter("track_min_hits").as_int());
          track_max_age_  = static_cast<int>(this->get_parameter("track_max_age").as_int());
          track_max_lost_ = static_cast<int>(this->get_parameter("track_max_lost").as_int());
          track_iou_gate_ = static_cast<float>(this->get_parameter("track_iou_gate").as_double());
          track_dist_gate_= static_cast<float>(this->get_parameter("track_dist_gate").as_double());
          if(track_enable_){
              RCLCPP_INFO(this->get_logger(),
                  "tracking: ON (min_hits=%d, max_age=%d, max_lost=%d, iou_gate=%.2f, dist_gate=%.1f)",
                  track_min_hits_,track_max_age_,track_max_lost_,
                  static_cast<double>(track_iou_gate_),static_cast<double>(track_dist_gate_));
          }else{
              RCLCPP_INFO(this->get_logger(),"tracking: OFF (track_enable=false)");
          }
          //打印启动信息
          RCLCPP_INFO(this->get_logger(),"BuoyDetectorNode started with synchronized input.");
          
        }
    private:
        //每帧调用一次，用滑动窗口算实时帧率（每 0.5 秒刷新一次，数字不会乱跳）
        void update_fps()
        {
            ++fps_frames_;
            const auto now=std::chrono::steady_clock::now();
            const double dt=std::chrono::duration<double>(now-fps_window_start_).count();
            if(dt>=0.5){
                fps_=fps_frames_/dt;
                fps_frames_=0;
                fps_window_start_=now;
            }
        }

        //把帧率画在右上角。字号随图像高度缩放，480p/720p/1080p 都好看。
        void draw_fps(cv::Mat &canvas,double fps)
        {
            const double scale=std::clamp(canvas.rows/720.0,0.5,2.0);
            //小分辨率下也保证有 2px 描边，不然字会显得很细
            const int thickness=std::max(2,static_cast<int>(std::round(scale*2)));

            char text[32];
            std::snprintf(text,sizeof(text),"FPS %.1f",fps);

            int baseline=0;
            const cv::Size size=cv::getTextSize(
                text,cv::FONT_HERSHEY_SIMPLEX,scale,thickness,&baseline);

            const int pad=std::max(4,static_cast<int>(std::round(scale*6)));
            const int margin=std::max(8,static_cast<int>(std::round(scale*12)));

            //文字左上角贴着右边距，基线距顶部一个文字高度
            const int tx=std::max(pad,canvas.cols-size.width-margin-pad);
            const int ty=std::max(size.height+pad,margin+size.height);

            //半透明黑底：不管背景多亮都看得清
            const cv::Rect box(std::max(0,tx-pad),
                               std::max(0,ty-size.height-pad),
                               std::min(canvas.cols,size.width+2*pad),
                               std::min(canvas.rows,size.height+baseline+2*pad));
            cv::rectangle(canvas,box,cv::Scalar(0,0,0),cv::FILLED);

            //黄色字，和绿色的检测框区分开
            cv::putText(canvas,text,cv::Point(tx,ty),
                cv::FONT_HERSHEY_SIMPLEX,scale,cv::Scalar(0,255,255),thickness,cv::LINE_AA);
        }

        //把检测框画到一张图上。只负责画，不参与任何检测计算。
        //注意：调用方必须传一张可以随便改的图（clone 过的），不要直接画在 cv_img 上。
        void draw_detections(cv::Mat &canvas,const std::vector<Detection> &detections)
        {
            for(const auto &det : detections){
                //bbox 是 [x_min, y_min, x_max, y_max]，全是 float
                const int x1=static_cast<int>(det.bbox[0]);
                const int y1=static_cast<int>(det.bbox[1]);
                const int x2=static_cast<int>(det.bbox[2]);
                const int y2=static_cast<int>(det.bbox[3]);

                //绿框
                cv::rectangle(canvas,cv::Point(x1,y1),cv::Point(x2,y2),
                    cv::Scalar(0,255,0),2);
                //中心点（红点）
                cv::circle(canvas,
                    cv::Point(static_cast<int>(det.center.x),static_cast<int>(det.center.y)),
                    3,cv::Scalar(0,0,255),-1);

                //左上角贴个标签：轨迹编号(如果有) + 类别 + 分数
                std::string label=det.class_name+" "+format_double(det.score,2);
                if(det.track_id >= 0){
                    //ID 是这条轨迹的"身份证"，连续帧之间不变才说明跟踪正常
                    label="ID"+std::to_string(det.track_id)+" "+label;
                }
                int baseline=0;
                const cv::Size text_size=cv::getTextSize(label,cv::FONT_HERSHEY_SIMPLEX,
                    0.5,1,&baseline);
                //小文本画面可能贴到图外，做一下边界保护
                const int tx=std::max(0,x1);
                const int ty=std::max(text_size.height+2,y1);
                cv::rectangle(canvas,
                    cv::Point(tx,ty-text_size.height-2),
                    cv::Point(tx+text_size.width+2,ty+baseline-2),
                    cv::Scalar(0,255,0),-1);
                cv::putText(canvas,label,cv::Point(tx+1,ty-1),
                    cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),1,cv::LINE_AA);
            }
        }

        // ==================== 跟踪主流程 ====================
        // 输入：这一帧的检测结果（会被就地写上 track_id / track_state）
        // 输出：这一帧"轨迹还活着、但没检测到"的预测框（只用于画图提示）
        //
        // 顺序很重要：先【预测】全部轨迹，再【关联】，最后【更新/新建/淘汰】。
        // 如果反过来先关联再预测，用的就是上一帧的位置，关联会一直慢一拍。
        std::vector<std::pair<cv::Rect2f,int>> update_tracks(
            std::vector<Detection> & dets,const rclcpp::Time & stamp)
        {
            std::vector<std::pair<cv::Rect2f,int>> missed_out;
            if(!track_enable_) return missed_out;

            // ---------- ① 所有轨迹先往前猜一步 ----------
            std::vector<cv::Rect2f> preds;
            preds.reserve(tracks_.size());
            for(auto & t : tracks_){
                preds.push_back(t->predict(stamp));
            }

            // ---------- ② 检测结果转成几何框 ----------
            // Detection 里是 [x1,y1,x2,y2]，这里转成"左上角+宽高"
            std::vector<cv::Rect2f> boxes;
            boxes.reserve(dets.size());
            for(const auto & d : dets){
                boxes.emplace_back(d.bbox[0],d.bbox[1],
                                   d.bbox[2]-d.bbox[0],d.bbox[3]-d.bbox[1]);
            }

            // ---------- ③ 数据关联：谁配谁 ----------
            // 做法：把所有"可能是一对"的组合列出来，按相似度从高到低挨个配。
            // 这叫贪心匹配。目标少的时候（比如现在 1 个浮标），
            // 它和"全局最优"的结果是一样的，没必要上更复杂的算法。
            std::vector<int>  det2track(dets.size(),-1);   // 每个检测配到了哪条轨迹
            std::vector<char> track_used(preds.size(),0);
            std::vector<char> det_used(dets.size(),0);

            struct Pair {int ti; int di; float iou;};
            std::vector<Pair> cand;
            for(size_t ti=0; ti<preds.size(); ++ti){
                for(size_t di=0; di<boxes.size(); ++di){
                    const float iou = box_iou(preds[ti],boxes[di]);

                    // 门限一：重叠太少，不可能是同一个目标。
                    // 没有门限的话，算法会硬把不相关的东西配成一对，
                    // 结果就是 ID 乱跳（这比不跟踪还糟）。
                    if(iou < track_iou_gate_) continue;

                    // 门限二：中心离太远也不许配。
                    // 主要防"画面整体平移"（船一转弯，所有框一起动）。
                    const float dx = (preds[ti].x + preds[ti].width *0.5f)
                                   - (boxes[di].x + boxes[di].width *0.5f);
                    const float dy = (preds[ti].y + preds[ti].height*0.5f)
                                   - (boxes[di].y + boxes[di].height*0.5f);
                    if(std::sqrt(dx*dx + dy*dy) > track_dist_gate_) continue;

                    cand.push_back({static_cast<int>(ti),static_cast<int>(di),iou});
                }
            }
            // 重叠越多，越可能是同一对，排前面
            std::sort(cand.begin(),cand.end(),
                [](const Pair & a,const Pair & b){return a.iou > b.iou;});

            for(const auto & c : cand){
                // 一对一：一条轨迹只能配一个检测，一个检测也只能配一条轨迹
                if(track_used[c.ti] || det_used[c.di]) continue;
                track_used[c.ti] = 1;
                det_used[c.di]   = 1;
                det2track[c.di]  = c.ti;
                tracks_[c.ti]->update(boxes[c.di],track_min_hits_);
            }

            // ---------- ④ 没配上的轨迹：记一笔"这帧没见到" ----------
            for(size_t ti=0; ti<preds.size(); ++ti){
                if(track_used[ti]) continue;
                auto & t = *tracks_[ti];
                t.mark_missed();
                // 已确认的轨迹，即使这帧没检测到，也把预测框留下来画图。
                // 这就是"跟踪能补漏检"的可视化：人能看到框还在，ID 没变。
                if(t.confirmed && t.misses <= track_max_age_){
                    missed_out.emplace_back(preds[ti],t.id);
                }
            }

            // ---------- ⑤ 没配上的检测：说明出现了新目标 ----------
            for(size_t di=0; di<boxes.size(); ++di){
                if(det_used[di]) continue;
                tracks_.push_back(std::make_unique<KalmanTrack>(
                    next_track_id_++,boxes[di],stamp));
                det2track[di] = static_cast<int>(tracks_.size()-1);
            }

            // ---------- ⑥ 把编号写回检测结果 ----------
            for(size_t di=0; di<dets.size(); ++di){
                const int ti = det2track[di];
                if(ti < 0) continue;
                dets[di].track_id = tracks_[ti]->id;
                dets[di].track_state = tracks_[ti]->confirmed
                    ? Detection::TRACK_CONFIRMED
                    : Detection::TRACK_TENTATIVE;
            }

            // ---------- ⑦ 淘汰 ----------
            // 两种情况删：
            //   * 从没确认过、又连续丢了 min_hits 次 —— 多半是单帧误检
            //   * 确认过但丢了太久（max_lost 帧）—— 目标真的走了
            tracks_.erase(
                std::remove_if(tracks_.begin(),tracks_.end(),
                    [this](const std::unique_ptr<KalmanTrack> & t){
                        if(!t->confirmed && t->misses >= track_min_hits_) return true;
                        return t->misses > track_max_lost_;
                    }),
                tracks_.end());

            return missed_out;
        }

        // 把"这一帧没检测到、但轨迹还在预测"的框画成橙色。
        // 只是给人看，不参与任何计算。
        void draw_missed_tracks(cv::Mat &canvas,
            const std::vector<std::pair<cv::Rect2f,int>> & missed) const
        {
            for(const auto & m : missed){
                const cv::Rect2f & b = m.first;
                cv::rectangle(canvas,
                    cv::Point(static_cast<int>(b.x),static_cast<int>(b.y)),
                    cv::Point(static_cast<int>(b.x+b.width),static_cast<int>(b.y+b.height)),
                    cv::Scalar(0,165,255),2);   // 橙色 = 预测（本帧没有检测到）
                cv::putText(canvas,"P"+std::to_string(m.second),
                    cv::Point(static_cast<int>(b.x),std::max(12,static_cast<int>(b.y)-6)),
                    cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,165,255),1,cv::LINE_AA);
            }
        }

        void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg,const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
        {
            auto start_time =this->now();
            update_fps();
            //
            DetectionArray output;
            output.header=img_msg->header;
            output.detections.clear();
            //检查分辨率是否匹配
            if(img_msg->width != info_msg->width || img_msg->height != info_msg->height){
                output.status=DetectionArray::STATUS_INPUT_ERROR;
                output.process_time_ms=0.0;
                rclcpp::Duration data_age= start_time -rclcpp::Time(img_msg->header.stamp);
                result_pub_->publish(output);
                RCLCPP_WARN(this->get_logger(),"INPUT_ERROR: Image resolution (%dx%d) does not match CameraInfo (%dx%d)", img_msg->width,img_msg->height,info_msg->width,info_msg->height);
                return;
            }
            //检测数据年龄
            rclcpp::Duration data_age=start_time-rclcpp::Time(img_msg->header.stamp);
            double stale_threshold= this->get_parameter("stale_threshold_sec").as_double();
            if(data_age.seconds()>stale_threshold){
                output.status=DetectionArray::STATUS_INPUT_STALE;
                output.data_age_ms=data_age.seconds()*1000.0;
                output.process_time_ms=0.0;
                result_pub_->publish(output);
                RCLCPP_WARN(this->get_logger(),"INPUT_STALE: age=%.2f s",data_age.seconds());
                save_stale_sample(img_msg,output,stale_threshold);
                return;

            }
            output.data_age_ms=data_age.seconds()*1000.0;
            //检查图像有效性
            cv::Mat cv_img;
            try{
                cv_img=cv_bridge::toCvShare(img_msg,"bgr8")->image;
            }
            catch(const cv_bridge::Exception& e){
                output.status=DetectionArray::STATUS_INPUT_ERROR;
                output.process_time_ms=0.0;
                result_pub_->publish(output);
                RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s",e.what());
                return;
            }
            if(cv_img.empty()){
                output.status=DetectionArray::STATUS_INPUT_ERROR;
                output.process_time_ms=0.0;
                result_pub_->publish(output);
                return;
            }
            //------------ 调用检测后端（hsv / yolo）------------
            //后端内部负责参数校验和推理；出错时抛异常，节点据此填状态码。
            //后端返回的 Detection 都是原图像素坐标，后半段不需要知道用的是哪种后端。
            std::vector<Detection> candidates;
            try{
                candidates=detector_->detect(cv_img);
            }
            catch(const moonwake_camera::DetectorConfigError & e){
                output.status=DetectionArray::STATUS_CONFIG_ERROR;
                output.process_time_ms=0.0;
                result_pub_->publish(output);
                RCLCPP_WARN(this->get_logger(),"CONFIG_ERROR: %s",e.what());
                save_config_error_sample(cv_img,img_msg,output,e.what());
                return;
            }
            catch(const moonwake_camera::DetectorModelError & e){
                //模型加载失败 / forward 抛异常：不能伪装成 NO_DETECTION
                output.status=DetectionArray::STATUS_MODEL_ERROR;
                output.process_time_ms=0.0;
                result_pub_->publish(output);
                RCLCPP_ERROR(this->get_logger(),"MODEL_ERROR: %s",e.what());
                return;
            }
            //------------ 跟踪：给检测结果分配 track_id ------------
            // 即使这一帧 candidates 是空的，也必须调用：
            // 要让已有轨迹"变老"，下一帧目标再出现时才能认出是同一个（ID 不变）。
            const auto missed_preds =
                update_tracks(candidates,rclcpp::Time(img_msg->header.stamp));

            //计算耗时（含检测 + 跟踪，也就是下游真正感受到的整帧耗时）
            auto end_time=this->now();
            output.process_time_ms=(end_time-start_time).seconds()*1000.0;
            if(candidates.empty()){
                output.status=DetectionArray::STATUS_NO_DETECTION;
            }else{
                output.status=DetectionArray::STATUS_DETECTED;
                output.detections=candidates;
            }
            result_pub_->publish(output);
            //没检测到目标时存一份样本：此时 mask（如果后端有）是真实的，最适合拿来调阈值
            if(output.status==DetectionArray::STATUS_NO_DETECTION){
                save_no_detection_sample(cv_img,detector_->debug_mask(),img_msg,output,
                                         detector_->last_reason());
            }
            //调试图发布（只有 HSV 后端有 mask；YOLO 后端 debug_mask() 返回空）
            bool pub_debug=this->get_parameter("publish_debug_mask").as_bool();
            cv::Mat mask=detector_->debug_mask();
            if (pub_debug && !mask.empty())  
            {
                auto debug_msg=cv_bridge::CvImage(
                    img_msg->header,"mono8",mask).toImageMsg();
                    debug_pub_->publish(*debug_msg);
                
            }
            //标注图发布：把检测框画在原图上，发到 debug 话题
            //必须先 clone：cv_img 是 toCvShare 出来的，和 img_msg 共享同一块内存，
            //直接往上画会把原始图像数据改坏
            if (this->get_parameter("publish_debug_image").as_bool())
            {
                cv::Mat annotated=cv_img.clone();
                draw_detections(annotated,candidates);
                //橙色框 = 本帧没检测到、但轨迹还在预测（跟踪在补漏检）
                draw_missed_tracks(annotated,missed_preds);
                draw_fps(annotated,fps_);   //右上角显示实时帧率
                auto annotated_msg=cv_bridge::CvImage(
                    img_msg->header,"bgr8",annotated).toImageMsg();
                debug_image_pub_->publish(*annotated_msg);
            }
            
            
        }

        //---------------- 异常样本保存 ----------------
        //把当前所有算法参数拍成一个快照，写进 txt 的 [params] 段。
        //用 value_to_string() 就不用为 int/double/bool 各写一遍取值代码。
        std::vector<std::pair<std::string,std::string>> snapshot_params()
        {
            std::vector<std::pair<std::string,std::string>> kv;
            //只能写「已经 declare_parameter 过的」参数名：
            //列表里出现未声明参数时，下面这行 get_parameter 会抛异常，保存样本时直接崩
            for (const char * name : {"detector_backend","h_min","h_max","s_min","s_max","v_min","v_max",
                                      "min_area","kernel_size","stale_threshold_sec",
                                      "adaptive_block_size","adaptive_C",
                                      "model_path","model_conf_threshold","model_iou_threshold",
                                      "model_input_size","model_num_threads","model_class_names",
                                      "track_enable","track_min_hits","track_max_age","track_max_lost",
                                      "track_iou_gate","track_dist_gate",
                                      "publish_debug_mask","publish_debug_image"}) {
                kv.emplace_back(name,this->get_parameter(name).value_to_string());
            }
            return kv;
        }

        //公共字段：状态码 / 状态名 / 阶段 / 原因 / 时间戳 / 参数快照
        FailureSample make_sample(const DetectionArray & output,const std::string & name,
                                  const std::string & stage,const std::string & reason,
                                  const builtin_interfaces::msg::Time & stamp)
        {
            FailureSample s;
            s.status_code=output.status;
            s.status_name=name;
            s.stage=stage;
            s.reason=reason;
            s.header_stamp=format_stamp(stamp);
            s.data_age_ms=output.data_age_ms;
            s.process_time_ms=output.process_time_ms;
            s.params=snapshot_params();
            return s;
        }

        //STATUS_INPUT_STALE：数据过期，在 OpenCV 处理之前就返回了
        void save_stale_sample(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
                               const DetectionArray & output,double stale_threshold_sec)
        {
            //先问记录器是否允许存：不允许就连图都不解码，省掉每帧的转换开销
            if(!failure_recorder_->should_record()) return;

            cv::Mat img;
            std::string decode_note="ok";
            try{
                //clone(): 让 Mat 拥有自己的内存，不依赖 img_msg 的生命周期
                img=cv_bridge::toCvShare(img_msg,"bgr8")->image.clone();
            }catch(const cv_bridge::Exception & e){
                decode_note=std::string("cv_bridge: ")+e.what();
            }

            FailureSample s=make_sample(output,"STATUS_INPUT_STALE","stale_check",
                "data_age "+format_double(output.data_age_ms)+" ms > stale_threshold "+
                format_double(stale_threshold_sec*1000.0)+" ms",
                img_msg->header.stamp);
            s.mask_source="placeholder: 过期帧在 OpenCV 处理前就 return 了，没有真实 mask";
            s.diagnostics={
                {"image_decode",decode_note},
                {"stale_threshold_sec",format_double(stale_threshold_sec)},
                {"params_applied","no(stale 帧没有进入 OpenCV)"}
            };
            failure_recorder_->record(img,cv::Mat(),s);
        }

        //STATUS_CONFIG_ERROR：参数非法，此时原图已经解码好了，可以直接存
        void save_config_error_sample(const cv::Mat & img,
                                      const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
                                      const DetectionArray & output,const std::string & why)
        {
            if(!failure_recorder_->should_record()) return;

            FailureSample s=make_sample(output,"STATUS_CONFIG_ERROR","config_check",
                "非法参数: "+why,img_msg->header.stamp);
            s.mask_source="placeholder: 参数非法，没有用它跑 OpenCV(kernel_size 非法时 getStructuringElement 会抛异常)";
            s.diagnostics={
                {"failed_checks",why},
                {"params_applied","no(参数非法，未执行检测)"}
            };
            failure_recorder_->record(img,cv::Mat(),s);
        }

        //STATUS_NO_DETECTION：整条 OpenCV 链路跑完了，mask 是真实结果
        void save_no_detection_sample(const cv::Mat & img,const cv::Mat & mask,
                                      const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
                                      const DetectionArray & output,const std::string & reason)
        {
            if(!failure_recorder_->should_record()) return;

            FailureSample s=make_sample(output,"STATUS_NO_DETECTION","detector_pipeline",
                reason,
                img_msg->header.stamp);
            //YOLO 后端不产生 mask，记录器会写一张同尺寸全黑图占位
            s.mask_source=mask.empty()
                ? "placeholder: 当前后端不产生 mask，写入同尺寸全黑图"
                : "pipeline: HSV 颜色阈值 + 自适应阈值 + 形态学的真实输出";
            s.diagnostics={
                {"backend",detector_->name()},
                {"params_applied","yes"}
            };
            failure_recorder_->record(img,mask,s);
        }

        message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        rclcpp::Publisher<DetectionArray>::SharedPtr result_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
        std::unique_ptr<moonwake_camera::FailureRecorder> failure_recorder_;
        //检测后端（hsv / yolo），由 detector_backend 参数决定
        std::unique_ptr<moonwake_camera::IDetector> detector_;

        //---------------- 跟踪 ----------------
        //参数在构造时读一次（不在每帧读，省开销；改完要重启节点）
        bool  track_enable_{true};
        int   track_min_hits_{3};
        int   track_max_age_{15};
        int   track_max_lost_{45};
        float track_iou_gate_{0.10f};
        float track_dist_gate_{200.0f};
        //所有活着的轨迹。用 unique_ptr 是因为 cv::KalmanFilter 不便宜，
        //而且 vector 扩容时指针不会失效
        std::vector<std::unique_ptr<KalmanTrack>> tracks_;
        int next_track_id_{1};   //只增不减，删掉的编号不会再复用
        //帧率统计（只用于 debug 标注图右上角的显示）
        std::chrono::steady_clock::time_point fps_window_start_{std::chrono::steady_clock::now()};
        int fps_frames_{0};
        double fps_{0.0};
        

};
int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<BuoyDetectorNode>());
    rclcpp::shutdown();
    return 0;
}