#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class SensorStatusNode : public rclcpp::Node {
public:
  SensorStatusNode() : Node("front_camera_status") {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(get_logger(), "front camera status (simulated): READY");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStatusNode>());
  rclcpp::shutdown();
  return 0;
}
