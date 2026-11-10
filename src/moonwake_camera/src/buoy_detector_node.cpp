#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "moonwake_camera/msg/detection_array.hpp"
#include "moonwake_camera/detector.hpp"
#include "moonwake_camera/failure_recorder.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono>
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
          //打印启动信息
          RCLCPP_INFO(this->get_logger(),"BuoyDetectorNode started with synchronized input.");
          
        }
    private:
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

                //左上角贴个标签：类别 + 分数
                const std::string label=det.class_name+" "+format_double(det.score,2);
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

        void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg,const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
        {
            auto start_time =this->now();
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
            //计算耗时
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
        

};
int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<BuoyDetectorNode>());
    rclcpp::shutdown();
    return 0;
}