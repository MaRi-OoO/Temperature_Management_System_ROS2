#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// using std::placeholders::_1;

class temperature_subscriber : public rclcpp::Node
{
public:
  temperature_subscriber()
  : Node("temperature_monitor_node")
  {
    auto topic_callback = [this](const std_msgs::msg::Float32 & msg){
      auto message = std_msgs::msg::String();
      if(msg.data >= 85){
        message.data = "Temperature is critical";
      }
      else if((msg.data >= 60) && (msg.data < 85)){
        message.data = "Temperature is high";
      }
      else{
        message.data = "Temperature is normal";
      }
      RCLCPP_INFO_STREAM(this->get_logger(), message.data);
      publisher_a->publish(message);
      publisher_b->publish(message);
    };
    subscription_a = this->create_subscription<std_msgs::msg::Float32>(
      "robot_a/temperature", 10, topic_callback);
    
    publisher_a = this->create_publisher<std_msgs::msg::String>("robot_a/status", 10);

    subscription_b = this->create_subscription<std_msgs::msg::Float32>(
      "robot_b/temperature", 10, topic_callback);

    publisher_b = this->create_publisher<std_msgs::msg::String>("robot_b/status", 10);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_a;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_b;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_a;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_b;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<temperature_subscriber>());
  rclcpp::shutdown();
  return 0;
}