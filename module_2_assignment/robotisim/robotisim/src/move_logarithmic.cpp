#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class LogarithmicSpiralNode : public rclcpp::Node
{
public:
    LogarithmicSpiralNode() : Node("logarithmic_spiral_turtle")
    {
        // Initialize parameters for the spiral
        a_ = 0.5;  // Spiral growth factor
        b_ = 0.1;  // Angular coefficient
        angular_velocity_ = 0.5;  // Angular velocity (radians/second)

        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Timer for publishing velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&LogarithmicSpiralNode::publish_velocity, this));

        RCLCPP_INFO(this->get_logger(), "Logarithmic spiral node started.");
    }

private:
    void publish_velocity()
    {
        // Calculate the current linear velocity based on the spiral equation
        double r = a_ * std::exp(b_ * theta_); // Radius at current angle
        double linear_velocity = r * angular_velocity_;

        // Publish velocity command
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_velocity;
        msg.angular.z = angular_velocity_;

        velocity_publisher_->publish(msg);

        // Increment the angle for the next step
        theta_ += angular_velocity_ * 0.1; // Update theta based on time step (dt = 0.1s)
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double a_;                  // Spiral growth factor
    double b_;                  // Angular coefficient
    double angular_velocity_;   // Angular velocity (radians/second)
    double theta_ = 0.0;        // Current angle in radians
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogarithmicSpiralNode>());
    rclcpp::shutdown();
    return 0;
}

