#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
// We initialize also the counter together with the node
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello CPP node");

        // Initialize timer_ (we use create_wall_timer function, then we need to bind the function to this class)
        // CTRL+SHIFT+I to align code like below
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello timer %d", counter_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

};

int main(int argc, char **argv)
{
    // :: scope resolution operator -> used to select what in the namespace I am using


    rclcpp::init(argc, argv);

    // auto -> get type automatically
    // In ros2 shared pointer are used to declare nodes (they are destroyed automatically)
    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}