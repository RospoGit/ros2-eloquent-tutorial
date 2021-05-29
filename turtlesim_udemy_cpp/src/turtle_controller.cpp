#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/available_turtles.hpp"
#include "math.h"
#include "geometry_msgs/msg/twist.hpp"

class TurtleControllerNode : public rclcpp::Node // MODIFY NAME
{
public:
    TurtleControllerNode() : Node("turtle_controller") // MODIFY NAME
    {

        // Get controlled turtle as parameter
        this->declare_parameter("controlled_turtle", "turtle1");
        controlled_turtle_ = this->get_parameter("controlled_turtle").as_string();

        // Subscribe to controlled turtle pose
        controlled_turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            controlled_turtle_ + "/pose", 10,
            std::bind(&TurtleControllerNode::subGetControlledTurtlePose, this, std::placeholders::_1));

        // Subscribe to list of available turtles

        available_turtles_sub_ = this->create_subscription<my_robot_interfaces::msg::AvailableTurtles>(
            "available_turtles", 10, std::bind(&TurtleControllerNode::subGetTargetPose, this, std::placeholders::_1));

        // Publish controlled turtle velocities
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(controlled_turtle_ + "/cmd_vel", 10);
        command_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                 std::bind(&TurtleControllerNode::turtleController, this));

        // Call kill client
        target_name_ = "";
        caught_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                std::bind(&TurtleControllerNode::checkIfTargetReached, this));

        RCLCPP_INFO(this->get_logger(), "Turtle_controller node has been started.");
    }

private:
    void subGetControlledTurtlePose(const turtlesim::msg::Pose::SharedPtr sub_pose_)
    {
        controlled_pose_.x = sub_pose_->x;
        controlled_pose_.y = sub_pose_->y;
        controlled_pose_.theta = sub_pose_->theta;
    }

    void subGetTargetPose(const my_robot_interfaces::msg::AvailableTurtles::SharedPtr avail_turtles_)
    {
        std::vector<double> turtle_distances_;
        if (avail_turtles_->name.size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "No available target.");
        }
        else
        {
            for (unsigned i = 0; i < avail_turtles_->name.size(); i++)
            {
                ;
                turtle_distances_.push_back(computeDistance(controlled_pose_.x, controlled_pose_.y,
                                                            avail_turtles_->x[i], avail_turtles_->y[i]));
            }

            int min_ix = computeIndexOfMinimum(turtle_distances_);

            target_pose_.x = avail_turtles_->x[min_ix];
            target_pose_.y = avail_turtles_->y[min_ix];
            target_name_ = avail_turtles_->name[min_ix];
        }
    }

    double computeDistance(double x1, double y1, double x2, double y2)
    {
        double deltaY = y2 - y1;
        double deltaX = x2 - x1;
        double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        return distance;
    }

    double computeAngle(double x1, double y1, double x2, double y2)
    {
        double deltaY = y2 - y1;
        double deltaX = x2 - x1;
        double angle = atan2(deltaY, deltaX);
        return angle;
    }

    int computeIndexOfMinimum(std::vector<double> vect)
    {
        int index = 0;

        double min_value = vect[0];

        for (int i = 0; i < vect.size(); i++)
        {
            if (vect[i] < min_value)
            {
                min_value = vect[i];
                index = i;
            }
        }

        return index;
    }

    void turtleController()
    {
        geometry_msgs::msg::Twist cmd_twist = geometry_msgs::msg::Twist();
        if (target_name_ == "")
        {
            cmd_twist.linear.x = 0.0;
            cmd_twist.angular.z = 0.0;
        }
        else
        {
            double dist = computeDistance(controlled_pose_.x, controlled_pose_.y, target_pose_.x, target_pose_.y);
            double angle = computeAngle(controlled_pose_.x, controlled_pose_.y, target_pose_.x, target_pose_.y);
            double deltaAngle = angle - controlled_pose_.theta;
            // Normalize
            if (deltaAngle >= pi)
            {
                deltaAngle -= 2 * pi;
            }
            else if (deltaAngle <= -pi)
            {
                deltaAngle += 2 * pi;
            }
            // Decide whether to stop
            if (dist < 0.1)
            {
                cmd_twist.linear.x = 0.0;
                cmd_twist.angular.z = 0.0;
            }
            else
            {
                cmd_twist.linear.x = 2 * dist;
                cmd_twist.angular.z = 6 * deltaAngle;
            }
        }
        cmd_vel_pub_->publish(cmd_twist);
    }

    void checkIfTargetReached()
    {
        if (target_name_.size() != 0)
        {
            double distance = computeDistance(controlled_pose_.x, controlled_pose_.y,
                                              target_pose_.x, target_pose_.y);
            if (distance < 0.1)
            {
                threads_.push_back(std::thread(std::bind(&TurtleControllerNode::callKillAndUpdateService, this, target_name_)));
                target_name_ = "";
            }
        }
    }

    void callKillAndUpdateService(std::string target)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill_and_update");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = target;

        auto future = client->async_send_request(request);
        // Here to wait for the future we use future.get() which waits until the future is completed
        try
        {
            // this will block the thread while waiting
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle %s has been killed.", target.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call to \\kill_and_update failed.");
        }
    }

    double pi = 3.1415926535897;

    std::string controlled_turtle_;
    turtlesim::msg::Pose controlled_pose_;
    turtlesim::msg::Pose target_pose_;
    std::string target_name_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr controlled_turtle_pose_sub_;
    rclcpp::Subscription<my_robot_interfaces::msg::AvailableTurtles>::SharedPtr available_turtles_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    rclcpp::TimerBase::SharedPtr caught_timer_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
