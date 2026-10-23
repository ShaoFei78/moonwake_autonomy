#include <chrono>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class ListenerNode : public rclcpp::Node {
    public:
    ListenerNode():Node("listener_node"){
        subscription_= this->create_subscription<std_msgs::msg::String>("chatter",10,[this](const std_msgs::msg::String::SharedPtr msg){
            this->latest_message_ = msg->data;
        });
    }
    private:
    std::string latest_message_; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}