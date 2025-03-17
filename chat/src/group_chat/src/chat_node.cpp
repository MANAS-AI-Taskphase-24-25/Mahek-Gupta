#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>

class ChatNode : public rclcpp::Node
{
public:
    ChatNode() : Node("chat_node")
    {
        pub1_ = this->create_publisher<std_msgs::msg::String>("chat1_topic", 10);
        pub2_ = this->create_publisher<std_msgs::msg::String>("chat2_topic", 10);

        sub1_ = this->create_subscription<std_msgs::msg::String>(
            "chat1_topic", 10, std::bind(&ChatNode::listener_callback1, this, std::placeholders::_1));

        sub2_ = this->create_subscription<std_msgs::msg::String>(
            "chat2_topic", 10, std::bind(&ChatNode::listener_callback2, this, std::placeholders::_1));

        input_thread_ = std::thread(&ChatNode::read_input, this);
    }

    ~ChatNode() {
        input_thread_.join();
    }

private:
    void listener_callback2(const std_msgs::msg::String &msg)
    {
        RCLCPP_INFO(this->get_logger(), "[Chat 1] %s", msg.data.c_str());
    }

    void listener_callback1(const std_msgs::msg::String &msg)
    {
        RCLCPP_INFO(this->get_logger(), "[Chat 2] %s", msg.data.c_str());
    }

    void read_input()
    {
        while (rclcpp::ok())
        {
            std::string message_text;
            std::cout << "Enter message (1 for Chat1, 2 for Chat2): ";
            std::getline(std::cin, message_text);

            if (message_text.empty()) continue;

            auto message = std_msgs::msg::String();
            message.data = message_text;

            if (message_text[0] == '1') {
                message.data = message_text.substr(2); // Remove "1 "
                RCLCPP_INFO(this->get_logger(), "You [Chat 1]: '%s'", message.data.c_str());
                pub1_->publish(message);
            } else if (message_text[0] == '2') {
                message.data = message_text.substr(2); // Remove "2 "
                RCLCPP_INFO(this->get_logger(), "You [Chat 2]: '%s'", message.data.c_str());
                pub2_->publish(message);
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid input! Start with '1 ' or '2 '.");
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
    std::thread input_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChatNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
