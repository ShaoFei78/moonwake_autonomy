#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
class TargetOffsetPublisher : public rclcpp::Node {
    public:
        TargetOffsetPublisher() : Node("target_offset_publisher"), index_{0}{
            this-> declare_parameter<double>("amplitude",0.5);
            double publish_hz= this-> declare_parameter<double>("publish_hz",1.0);
            if (publish_hz<=0)
            {
                RCLCPP_WARN(this->get_logger(),"publish_hz must be positive");
                publish_hz = 1.0;
            }
            
            auto period = std::chrono::duration<double>(1.0 / publish_hz);
            publisher_=this->create_publisher<std_msgs::msg::Float64>("target_offset",10);
            timer_=this->create_wall_timer(period,std::bind(&TargetOffsetPublisher::timer_callback,this));


        }
    private:
        void timer_callback() {
            double amp;
            this->get_parameter("amplitude", amp);
            if (amp < 0.0) {
            amp = std::abs(amp);
                }
            double offset = kCoefficients[index_] * amp;
            auto msg = std_msgs::msg::Float64();
        msg.data = offset;
        publisher_->publish(msg);
        index_ = (index_ + 1) % kSequenceLength;
        RCLCPP_INFO(this->get_logger(),
            "Published offset: %.3f (index=%zu, amplitude=%.3f)",
            offset, index_, amp);
        }
        static constexpr size_t kSequenceLength = 5;
    static constexpr double kCoefficients[kSequenceLength] = {
        -1.0, -0.5, 0.0, 0.5, 1.0
    };
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t index_;

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetOffsetPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}