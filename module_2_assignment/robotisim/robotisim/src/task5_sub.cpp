#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class LEDSubscriber : public rclcpp::Node
{
public:
    LEDSubscriber() : Node("led_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "led_states", 10,
            std::bind(&LEDSubscriber::led_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "LED Subscriber node has been started.");
    }

private:
    void led_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received LED states: [%s]", array_to_string(msg->data).c_str());
        for (size_t i = 0; i < msg->data.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "LED %zu: %s", i + 1, msg->data[i] ? "ON" : "OFF");
        }
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

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LEDSubscriber>());
    rclcpp::shutdown();
    return 0;
}

