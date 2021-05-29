#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedPanelNode : public rclcpp::Node // MODIFY NAME
{
public:
    LedPanelNode() : Node("led_panel") // MODIFY NAME
    {
        //led_states_ = {false, false, false};
        this->declare_parameter("led_states", std::vector<bool>{false, false, false});
        led_states_ = this->get_parameter("led_states").as_bool_array();

        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_states", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(4),
                                         std::bind(&LedPanelNode::callback_led_states, this));
        server_ = this->create_service<
            my_robot_interfaces::srv::SetLed>("set_led",
                                              std::bind(&LedPanelNode::callback_set_led, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Led Panel has been launched.");
    }

private:
    void callback_led_states()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_states = led_states_;
        publisher_->publish(msg);
    }
    void callback_set_led(const my_robot_interfaces::srv::SetLed_Request::SharedPtr request,
                          const my_robot_interfaces::srv::SetLed_Response::SharedPtr response)
    {
        if (request->led_number >= 0 && request->led_number <= 3){
            led_states_[request->led_number -1] = request->led_state;
            response->success = true;
        }
        else{
            response->success = false;
        }
        callback_led_states();
    }

    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr publisher_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<bool> led_states_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
