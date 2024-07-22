#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include "turtle_interfaces/msg/turtle_vector.hpp"
#include "turtle_interfaces/srv/add_turtle.hpp"

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>
        (
            "turtle1/pose",
            10,
            std::bind(&TurtleControllerNode::poseCallback, this, std::placeholders::_1)
        );
        add_turtle_server_ = this->create_service<turtle_interfaces::srv::AddTurtle>
        (
            "add_turtle",
            std::bind(&TurtleControllerNode::addTurtleCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::timerCallback, this));

    }

private:
    void poseCallback(const turtlesim::msg::Pose& pose)
    {
        current_pose_= pose;

        timerCallback();
    }

    void addTurtleCallback(const turtle_interfaces::srv::AddTurtle::Request::SharedPtr request,
                           const turtle_interfaces::srv::AddTurtle::Response::SharedPtr response)
    {
        response->success = true;

        auto turtle = turtle_interfaces::msg::Turtle();
        turtle.name = request->name;
        turtle.x = request->x;
        turtle.y = request->y;

        alive_turtles_.push_back(turtle);

        RCLCPP_INFO(this->get_logger(), "Request from client to add turtle %s", request->name.c_str());
    }

    void timerCallback()
    {
        if(alive_turtles_.size() == 0)
            return;

        double dist_x = alive_turtles_.front().x - current_pose_.x;
        double dist_y = alive_turtles_.front().y - current_pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto twist = geometry_msgs::msg::Twist();

        if(distance > 0.5)
        {
            // Get closer to target
            twist.linear.x = 2 * distance;

            double angle_diff = std::atan2(dist_y, dist_x) - current_pose_.theta;

            // Normalize the angle difference to the range [-PI, PI]
            if(angle_diff > M_PI) angle_diff -= 2 * M_PI;
            if(angle_diff < -M_PI) angle_diff += 2 * M_PI;

            twist.angular.z = 5 * angle_diff;
        }
        else
        {
            // Target has been reached

            // Stop movement
            twist.linear.x = 0;
            twist.angular.z = 0;

            // Call the /kill service to kill the turtle
            callKillService();

            alive_turtles_.pop_front();

            RCLCPP_INFO(this->get_logger(), "Target Reached");
        }

        twist_pub_->publish(twist);
    }

    void callKillService()
    {
        while(!kill_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the kill service to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = alive_turtles_.front().name;

        auto future = kill_client_->async_send_request(request, [this, request](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
        {
            handleKillResponse(future, request->name);
        });
        
    }

    void handleKillResponse(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future, std::string name)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle %s has been killed", name.c_str());
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call kill service");
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Service<turtle_interfaces::srv::AddTurtle>::SharedPtr add_turtle_server_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose current_pose_;
    turtlesim::msg::Pose goal_pose_;

    std::list<turtle_interfaces::msg::Turtle> alive_turtles_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}