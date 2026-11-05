#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "moonwake_camera/msg/detection_array.hpp"
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
          this->declare_parameter<int>("h_min", 0);
          this->declare_parameter<int>("h_max", 180);
          this->declare_parameter<int>("s_min", 50);
          this->declare_parameter<int>("s_max", 255);
          this->declare_parameter<int>("v_min", 50);
          this->declare_parameter<int>("v_max", 255);
          this->declare_parameter<int>("min_area", 100);
          this->declare_parameter<int>("kernel_size", 5);
          this->declare_parameter<double>("stale_threshold_sec", 0.5);
          this->declare_parameter<bool>("publish_debug_mask", false);
          //异常样本保存相关参数
          this->declare_parameter<bool>("save_failure_samples", true);
          //留空 = 自动存到 <本包目录>/failure_samples/（见 default_failure_dir）
          //填了路径就用你给的路径，相对路径相对"启动节点时的当前目录"
          this->declare_parameter<std::string>("failure_dir", "");
          this->declare_parameter<double>("failure_min_interval_sec", 10.0);
          this->declare_parameter<int>("failure_max_samples", 200);
          //创建发布者
          result_pub_=this->create_publisher<DetectionArray>("detection",10);
          debug_pub_=this->create_publisher<sensor_msgs::msg::Image>("debug_mask",10);
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
          RCLCPP_INFO(this->get_logger(),"BuoyDetectorNode started with synchronized input.");
          
        }
    private:
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
            //检查配置有效性
            int h_min=this->get_parameter("h_min").as_int();
            int h_max=this->get_parameter("h_max").as_int();
            int s_min=this->get_parameter("s_min").as_int();
            int s_max=this->get_parameter("s_max").as_int();
            int v_min=this->get_parameter("v_min").as_int();
            int v_max=this->get_parameter("v_max").as_int();
            int min_area=this->get_parameter("min_area").as_int();
            int kernel_size=this->get_parameter("kernel_size").as_int();
            if (h_min>=h_max||s_min>=s_max||v_min>=v_max||min_area<=0||kernel_size<=0)
            {
              //把到底哪一项不合法写清楚，方便事后看 txt 定位
              std::string why;
              if (h_min>=h_max) why+="h_min>=h_max; ";
              if (s_min>=s_max) why+="s_min>=s_max; ";
              if (v_min>=v_max) why+="v_min>=v_max; ";
              if (min_area<=0) why+="min_area<=0; ";
              if (kernel_size<=0) why+="kernel_size<=0; ";
              if (why.size()>=2) why.erase(why.size()-2);  //去掉结尾多出来的 "; "
              output.status=DetectionArray::STATUS_CONFIG_ERROR;
              output.process_time_ms=0.0;
              result_pub_->publish(output);
              RCLCPP_WARN(this->get_logger(),"CONFIG_ERROR:invalid parameters");
              save_config_error_sample(cv_img,img_msg,output,why);
              return;
            }
            //
            //OpenCV处理
            cv::Mat hsv,mask;
            cv::cvtColor(cv_img,hsv,cv::COLOR_BGR2HSV);
            //
            cv::Scalar lower(h_min,s_min,v_min);
            cv::Scalar upper(h_max,s_max,v_max);
            cv::inRange(hsv,lower,upper,mask);
            //
            cv::Mat kernel=cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(kernel_size,kernel_size));
            cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernel);
            cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernel);
            //
            cv::Mat labels,stats,centroids;
            int n_labels=cv::connectedComponentsWithStats(mask,labels,stats,centroids);
            //
            std::vector<Detection> candidates;
            for(int i=1;i<n_labels;++i){
                int area=stats.at<int>(i,cv::CC_STAT_AREA);
                if(area<min_area)continue;
                Detection det;
                det.class_name="buoy";
                det.center.x=centroids.at<double>(i,0);
                det.center.y=centroids.at<double>(i,1);
                det.center.z=0.0;
                int x =stats.at<int>(i,cv::CC_STAT_LEFT);
                int y =stats.at<int>(i,cv::CC_STAT_TOP);
                int w =stats.at<int>(i,cv::CC_STAT_WIDTH);
                int h =stats.at<int>(i,cv::CC_STAT_HEIGHT);
                det.bbox={
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(w+x),
                    static_cast<float>(h+y)

                };
                //
                float bbox_area =static_cast<float>(w*h);
                det.score=(bbox_area>0)?static_cast<float>(area)/bbox_area:0.0f;
                candidates.push_back(det);

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
            //没检测到目标时存一份样本：此时 mask 是真实的，最适合拿来调阈值
            if(output.status==DetectionArray::STATUS_NO_DETECTION){
                save_no_detection_sample(cv_img,mask,img_msg,output,n_labels-1,min_area);
            }
            //调试图发布
            bool pub_debug=this->get_parameter("publish_debug_mask").as_bool();
            if (pub_debug)  
            {
                auto debug_msg=cv_bridge::CvImage(
                    img_msg->header,"mono8",mask).toImageMsg();
                    debug_pub_->publish(*debug_msg);
                
            }
            
            
        }

        //---------------- 异常样本保存 ----------------
        //把当前所有算法参数拍成一个快照，写进 txt 的 [params] 段。
        //用 value_to_string() 就不用为 int/double/bool 各写一遍取值代码。
        std::vector<std::pair<std::string,std::string>> snapshot_params()
        {
            std::vector<std::pair<std::string,std::string>> kv;
            for (const char * name : {"h_min","h_max","s_min","s_max","v_min","v_max",
                                      "min_area","kernel_size","stale_threshold_sec",
                                      "publish_debug_mask"}) {
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
                                      const DetectionArray & output,int components,int min_area)
        {
            if(!failure_recorder_->should_record()) return;

            FailureSample s=make_sample(output,"STATUS_NO_DETECTION","opencv_pipeline",
                "mask 里有 "+std::to_string(components)+" 个连通域，但每个面积都 < min_area="+
                std::to_string(min_area),
                img_msg->header.stamp);
            s.mask_source="pipeline: inRange + 形态学的真实输出";
            s.diagnostics={
                {"components_total",std::to_string(components)},
                {"components_passed_min_area","0"},
                {"params_applied","yes"}
            };
            failure_recorder_->record(img,mask,s);
        }

        message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        rclcpp::Publisher<DetectionArray>::SharedPtr result_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
        std::unique_ptr<moonwake_camera::FailureRecorder> failure_recorder_;
        

};
int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<BuoyDetectorNode>());
    rclcpp::shutdown();
    return 0;
}