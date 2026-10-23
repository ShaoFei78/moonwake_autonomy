#include <chrono>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class TalkerNode : public rclcpp::Node {
    public:
    TalkerNode():Node("talker_node"){
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);
        timer_= this->create_wall_timer(500ms,[this](){
            auto msg=std_msgs::msg::String();
            msg.data="Hello,ShaoFei.";
            publisher_->publish(msg);
            
        });
    }
    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}