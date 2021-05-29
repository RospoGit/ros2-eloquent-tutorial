#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client") // MODIFY NAME
    {   
        // We call the service in a new thread, that could possibly get stuck, but we 
        // can continue spinning
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 5, 7)));
    }

    void callAddTwoIntsService(int a, int b)
    {   
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);
        // Here to wait for the future we use future.get() which waits until the future is completed
        try
        {
            // this will block the thread while waiting
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a,b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

private:
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
