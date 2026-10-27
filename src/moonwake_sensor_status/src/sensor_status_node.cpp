#include <chrono>
#include <memory>
#include <std_srvs/srv/set_bool.hpp>

#include "rclcpp/rclcpp.hpp"

class SensorStatusNode : public rclcpp::Node {
public:
  SensorStatusNode() : Node("front_camera_status") {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
     if (detailed_log_enabled_) {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] front camera status: READY | FPS: 30 , Temp: 45C , Latency:12ms");
      } else {
        RCLCPP_INFO(this->get_logger(), "front camera status: READY");
      }
    });
    // Create a service to enable/disable detailed logging
    service_=this->create_service<std_srvs::srv::SetBool>(
      "set_detailed_log",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request>request,
        std::shared_ptr<std_srvs::srv::SetBool::Response>response) {
          detailed_log_enabled_ = request->data;
          response->success = true;
          response->message = detailed_log_enabled_ ? "Detailed logging enabled":"Detailed logging disabled";
        }
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::Service::SharedPtr service_;
  bool detailed_log_enabled_ = false;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStatusNode>());
  rclcpp::shutdown();
  return 0;
}
