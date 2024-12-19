#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class LEDPublisher : public rclcpp::Node
{
public:
    LEDPublisher() : Node("led_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("led_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LEDPublisher::publish_led_states, this));
        RCLCPP_INFO(this->get_logger(), "LED Publisher node has been started.");
    }

private:
    void publish_led_states()
    {
        auto message = std_msgs::msg::UInt8MultiArray();
        message.data = {1, 0, 1, 1, 0}; // Example LED states
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published LED states: [%s]", array_to_string(message.data).c_str());
    }

    std::string array_to_string(const std::vector<uint8_t> &data)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < data.size(); ++i)
        {
            oss << static_cast<int>(data[i]);
            if (i < data.size() - 1)
                oss << ", ";
        }
        return oss.str();
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LEDPublisher>());
    rclcpp::shutdown();
    return 0;
}

