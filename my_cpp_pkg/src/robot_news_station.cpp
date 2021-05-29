#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station")
    {   
        this->declare_parameter("robot_name", "R2D");
        robot_name_ = this->get_parameter("robot_name").as_string();
        publisher_ = this->create_publisher<std_msgs::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
    }

private:
    void publishNews()
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the Robot News Station");
        publisher_->publish(msg);
    }
    std::string robot_name_;
    // We have to declare the publisher before creating
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}