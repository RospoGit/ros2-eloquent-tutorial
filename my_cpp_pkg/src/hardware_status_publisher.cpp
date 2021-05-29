#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
/* To make VS Code recognise the imported msg, 
add in .vscode/c_cpp_properties.json the path ~/ros2_ws/install/my_robot_interfaces/include
*/

class HardwareStatusPublisherNode : public rclcpp::Node // MODIFY NAME
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher") // MODIFY NAME
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(
            "hardware_status", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(
                                         &HardwareStatusPublisherNode::publish_hardware_status, this));
        RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher has been started.");
    }

private:
    void publish_hardware_status()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 57;
        msg.are_motors_ready = true;
        msg.debug_message = "Motors are too hot!";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
