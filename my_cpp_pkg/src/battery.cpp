#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node // MODIFY NAME
{
public:
    BatteryNode() : Node("battery") // MODIFY NAME
    {   
        low_on_battery_ = false;
        last_change_time_ = this->get_clock()->now().seconds();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&BatteryNode::callback_set_led, this));
        RCLCPP_INFO(this->get_logger(), "Battery node has been launched.");
    }

private:
    void call_set_led(int led_number, bool led_state)
    {
        auto client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }
        auto request_ = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request_->led_number = led_number;
        request_->led_state = led_state;

        auto future = client_->async_send_request(request_);
        try
        {
            auto response = future.get();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void callback_set_led()
    {   double time_now_ = this->get_clock()->now().seconds();
        if (low_on_battery_ && (time_now_-last_change_time_) > 6.0){
            low_on_battery_ = false;
            RCLCPP_INFO(this->get_logger(), "Battery charged!");
            threads_.push_back(std::thread(std::bind(&BatteryNode::call_set_led, this, 3, false)));
            last_change_time_ = time_now_;
        }else if (!low_on_battery_ && (time_now_-last_change_time_) > 4){
            low_on_battery_ = true;
            RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging...");
            threads_.push_back(std::thread(std::bind(&BatteryNode::call_set_led, this, 3, true)));
            last_change_time_ = time_now_;
        }
        
    }

    std::vector<std::thread> threads_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_change_time_;
    bool low_on_battery_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
