#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtle_interfaces/msg/turtle.hpp"
#include "turtle_interfaces/msg/turtle_vector.hpp"
#include "turtle_interfaces/srv/add_turtle.hpp"

#include <random>

constexpr double lower_bound = 1.0;
constexpr double upper_bound = 9.0;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode()
        : Node("turtle_spawner"),
          uniform_coordinate_distribution_(lower_bound, upper_bound)
    {
        spawn_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                               std::bind(&TurtleSpawnerNode::spawnTurtleCallback, this));
        
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        add_turtle_client_ = this->create_client<turtle_interfaces::srv::AddTurtle>("add_turtle");
    }

private:

    void spawnTurtleCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Spawning turtle");
        while(!spawn_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        // Get random location on the canvas
        request->x = uniform_coordinate_distribution_(random_engine_);
        request->y = uniform_coordinate_distribution_(random_engine_);

        // Call `spawn` service to randomly spawn a turtle
        auto future = spawn_client_->async_send_request(request, [this, request](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
        {
            handleSpawnResponse(future, request);
        });
    }

    void handleSpawnResponse(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future, turtlesim::srv::Spawn::Request::SharedPtr request)
    {
        try
        {
            auto response = future.get();
            if (!response->name.empty())
            {
                callAddTurtleService(response->name, request->x, request->y);
                RCLCPP_INFO(this->get_logger(), "Turtle %s has been spawned", response->name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void callAddTurtleService(std::string_view name, const float& x, const float& y)
    {
        while (!add_turtle_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for add_turtle service to be up...");
        }

        auto request = std::make_shared<turtle_interfaces::srv::AddTurtle::Request>();
        request->name = name;
        request->x = x;
        request->y = y;

        auto future = add_turtle_client_->async_send_request(request, [this](rclcpp::Client<turtle_interfaces::srv::AddTurtle>::SharedFuture future)
        {
            handleAddTurtleResponse(future);
        });
    }

    void handleAddTurtleResponse(rclcpp::Client<turtle_interfaces::srv::AddTurtle>::SharedFuture future)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Call add_turtle success: %s", response->success ? "true" : "false");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

private:
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtle_interfaces::srv::AddTurtle>::SharedPtr add_turtle_client_;

    std::uniform_real_distribution<double> uniform_coordinate_distribution_;
    std::default_random_engine random_engine_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}