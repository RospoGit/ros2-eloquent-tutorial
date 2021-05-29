#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/available_turtles.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TurtleSpawnNode : public rclcpp::Node // MODIFY NAME
{
public:
    TurtleSpawnNode() : Node("turtle_spawn") // MODIFY NAME
    {

        // Call spawn server
        spawn_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                               std::bind(&TurtleSpawnNode::SpawnServiceHandler, this));

        // List of Available Turtles
        available_turtles_ = my_robot_interfaces::msg::AvailableTurtles();

        // Kill server (used to call kill service from turtlesim)
        kill_and_update_server_ = this->create_service<turtlesim::srv::Kill>(
            "kill_and_update", std::bind(&TurtleSpawnNode::kill_and_update, this, _1, _2));

        // Publish list of available turtles
        available_turtle_pub_ = this->create_publisher<my_robot_interfaces::msg::AvailableTurtles>(
            "available_turtles", 10);
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TurtleSpawnNode::publishAvailableTurtles, this));

        RCLCPP_INFO(this->get_logger(), "Turtle spawn node has been started.");
    }

private:
    

    struct TurtlePositionStruct
    {
        float x, y, theta;
    };

    void SpawnServiceHandler()
    {
        spawn_threads_.push_back(std::thread(std::bind(
            &TurtleSpawnNode::callSpawnService, this)));
    }

    void callSpawnService()
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        TurtlePositionStruct generated_coordinates = generateRandomPosition(0, 11);
        request->x = generated_coordinates.x;
        request->y = generated_coordinates.y;
        request->theta = generated_coordinates.theta;
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),
                        "Turtle %s has been spawned at %f %f",
                        response->name.c_str(), generated_coordinates.x, generated_coordinates.y);
            add_turtle_to_list(response->name, generated_coordinates.x, generated_coordinates.y);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call to Spawn failed.");
        }
    }

    TurtlePositionStruct generateRandomPosition(int min_value, int max_value)
    {
        int deltaXY = max_value - min_value;
        int x = (rand() % deltaXY) + min_value;
        int y = (rand() % deltaXY) + min_value;
        float theta = (rand() % 360) * 2 * pi / 360;
        if (theta > pi)
        {
            theta -= 2 * pi;
        }
        TurtlePositionStruct position = {(float)x, (float)y, (float)theta};
        return position;
    }

    void add_turtle_to_list(std::string name, double x, double y)
    {
        available_turtles_.name.push_back(name);
        available_turtles_.x.push_back(x);
        available_turtles_.y.push_back(y);
    }

    void remove_turtle_from_list(std::string name)
    {
        int vector_size_ = available_turtles_.name.size();
        int i;
        for (i=0; i<vector_size_; i++){
            if (available_turtles_.name[i] == name){
                available_turtles_.name.erase(available_turtles_.name.begin()+i);
                available_turtles_.x.erase(available_turtles_.x.begin()+i);
                available_turtles_.y.erase(available_turtles_.y.begin()+i);
                break;
            }
            
        }
        publishAvailableTurtles();
    }

    void kill_and_update(const turtlesim::srv::Kill::Request::SharedPtr request,
                         const turtlesim::srv::Kill::Response::SharedPtr response)
    {
        remove_turtle_from_list(request->name);
        kill_threads_.push_back(std::thread(std::bind(&TurtleSpawnNode::callKillService, this, request->name)));
    }

    void callKillService(std::string name)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),
                        "Turtle %s has been killed.", name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call to Kill failed.");
        }
    }

    void publishAvailableTurtles()
    {
        auto msg = my_robot_interfaces::msg::AvailableTurtles();
        msg = available_turtles_;
        available_turtle_pub_->publish(msg);
    }

    double pi = 3.1415926535897;
    // std::vector<std::string> available_turtles_names_;
    // std::vector<double> available_turtles_y;
    // std::vector<double> available_turtles_x_;
    my_robot_interfaces::msg::AvailableTurtles available_turtles_;

    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::vector<std::thread> spawn_threads_;
    std::vector<std::thread> kill_threads_;
    rclcpp::Publisher<my_robot_interfaces::msg::AvailableTurtles>::SharedPtr available_turtle_pub_;
    rclcpp::Service<turtlesim::srv::Kill>::SharedPtr kill_and_update_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
