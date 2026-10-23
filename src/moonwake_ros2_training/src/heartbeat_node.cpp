#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node {
public:
  HeartbeatNode() : Node("heartbeat_node") {
    timer_ = create_wall_timer(1s, [this]() {
      RCLCPP_INFO(get_logger(), "node is alive");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatNode>());
  rclcpp::shutdown();
  return 0;
}