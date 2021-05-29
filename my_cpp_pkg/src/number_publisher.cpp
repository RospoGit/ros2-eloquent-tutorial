#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {   
        this->declare_parameter("number_to_publish", 4);
        this->declare_parameter("publish_frequency", 1.0);

        /* In cpp I have also to cast the param type in order to match the declared
        type of the variable that is getting assigned */
        number_ = this->get_parameter("number_to_publish").as_int();
        double publish_frequency_ = this->get_parameter("publish_frequency").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_frequency_)),
                                         std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started.");
    }

private:
    void publishNumber()
    {
        auto num = std_msgs::msg::Int64();
        num.data = number_;
        publisher_->publish(num);
    }

    // We have to declare the publisher before creating
    int number_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}