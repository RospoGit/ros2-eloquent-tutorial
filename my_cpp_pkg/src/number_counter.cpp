#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        //counter_.data = 0;
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::callbackCountNumber, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number_count", 10);
        server_ = this->create_service<std_srvs::srv::SetBool>(
            "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
            
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    
    void callbackCountNumber(const std_msgs::msg::Int64::SharedPtr num)
    {
        counter_ += num->data;
        // -> is equal to (*...). . Here I have the pointer to num, so I have to use ->
        //RCLCPP_INFO(this->get_logger(), std::to_string(counter_.data));
        auto new_msg = std_msgs::msg::Int64();
        new_msg.data = counter_;
        publisher_->publish(new_msg);
        RCLCPP_INFO(this->get_logger(), "Counter: %d", new_msg.data);
    }
    
    void callbackResetCounter(const std_srvs::srv::SetBool_Request::SharedPtr request,
                              const std_srvs::srv::SetBool_Response::SharedPtr response)
    {   
        
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
        }
        else{
            response->success = false;
        }
        RCLCPP_INFO(this->get_logger(), "Reset Counter: %d", request->data);
        
    }
    

    // Declare
    //std_msgs::msg::Int64 counter_;
    int counter_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}