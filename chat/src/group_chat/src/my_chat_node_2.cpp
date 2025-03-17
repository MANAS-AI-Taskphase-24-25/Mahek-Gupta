#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>

class ChatNode2 : public rclcpp::Node
{
public:
    ChatNode2() : Node("chat_node_2")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chat2_topic", 10);

        using std::placeholders::_1;
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chat1_topic", 10, std::bind(&ChatNode2::listener_callback, this, _1));

        input_thread_ = std::thread(&ChatNode2::read_input, this);
    }

    ~ChatNode2() {
        input_thread_.join();
    }

private:
    void listener_callback(const std_msgs::msg::String &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Chat1: %s", msg.data.c_str());
    }

    void read_input()
    {
        while (rclcpp::ok())
        {
            std::string message_text;
            std::cout << "You (Chat2): ";
            std::getline(std::cin, message_text);

            auto message = std_msgs::msg::String();
            message.data = message_text;

            RCLCPP_INFO(this->get_logger(), "Sending: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::thread input_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChatNode2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
