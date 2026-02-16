#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
    explicit Talker(const std::string & topic_name)
    : Node("talker")
    {
        auto publish_message = 
        [this]() -> void
        {
            auto msg = std::make_unique<geometry_msgs::msg::Twist>();

            msg->linear.x = 0.0;
            msg->linear.y = 0.0;
            msg->linear.z = 0.0;

            msg->angular.x = 0.0;
            msg->angular.y = 0.0;
            msg->angular.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Stop_command");
            pub_->publish(std::move(msg));
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_name, qos);

        timer_ = create_wall_timer(100ms, publish_message);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Talker>("cmd_vel");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}


