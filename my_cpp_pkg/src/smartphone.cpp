#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        // Initialize subscriber (topic_name, queue_size, callback).
        // we need to bind the callback (&Sma...News) the smartphone node (this). Then we need to place
        // a placeholder_1 because we have one param in the callback (const ... msg)
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_news", 10,
            std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }

private:
    // callback that is called by the subs. We need a shared pointer with const type.
    void callbackRobotNews(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }
    // Create subscriber (also this a shared pointer)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}