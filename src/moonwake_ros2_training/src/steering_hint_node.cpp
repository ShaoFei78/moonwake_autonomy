#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
class SteeringHintNode : public rclcpp::Node {
    public:
          SteeringHintNode():Node("steering_hint_node"){
            this->declare_parameter<double>("deadband",0.05);
            subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
                "target_offset",10,std::bind(&SteeringHintNode::callback,this,std::placeholders::_1)
            );
            publisher_ = this->create_publisher<std_msgs::msg::String>("steering_hint",10);
            RCLCPP_INFO(this->get_logger(), "Steering Hint Node started.");
          };
    private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    void callback(const std_msgs::msg::Float64::SharedPtr msg){
        double db;
        this-> get_parameter("deadband",db);
        double current_offset = msg->data;
        std_msgs::msg::String result;
        if (current_offset<-db)  
        {
            result.data="LEFT";
            
        }else if (current_offset>db)
        {
            result.data="RIGHT";
            
        }else{
            result.data="STRAIGHT";
           
        }
        publisher_->publish(result);
        RCLCPP_INFO(this->get_logger(), "Offset: %.2f -> Hint: %s", current_offset, result.data.c_str());
        
         
    }

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SteeringHintNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
