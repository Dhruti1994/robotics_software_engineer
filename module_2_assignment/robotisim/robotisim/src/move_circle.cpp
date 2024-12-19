#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CirclePathNode : public rclcpp::Node
{
public:
    CirclePathNode() : Node("circle_path_turtle")
    {
        // Hardcoded radius value
        radius_ = 2.0;

        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Timer for publishing velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CirclePathNode::publish_velocity, this));

        // Calculate the linear and angular velocities
        linear_velocity_ = 1.0; // meters/second
        angular_velocity_ = linear_velocity_ / radius_;

        RCLCPP_INFO(this->get_logger(), "Turtle will move in a circle with radius %.2f meters", radius_);
    }

private:
    void publish_velocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_velocity_;
        msg.angular.z = angular_velocity_;

        velocity_publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double radius_;
    double linear_velocity_;
    double angular_velocity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CirclePathNode>());
    rclcpp::shutdown();
    return 0;
}

