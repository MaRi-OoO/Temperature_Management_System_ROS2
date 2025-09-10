#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class temperature_publisher : public rclcpp::Node
{
public:
  temperature_publisher()
  : Node("temperature_sensor_node"), count_(0)
  {
    publisher_a = this->create_publisher<std_msgs::msg::Float32>("robot_a/temperature", 10);
    publisher_b = this->create_publisher<std_msgs::msg::Float32>("robot_b/temperature", 10);

    auto timer_callback = [this](){
      static auto message = std_msgs::msg::Float32();
      message.data += 10.0;
      
      RCLCPP_INFO_STREAM(this->get_logger(), "robot_a temperature => '" << message.data << "'");
      RCLCPP_INFO_STREAM(this->get_logger(), "robot_b temperature => '" << message.data << "'");
      
      publisher_a->publish(message);
      publisher_b->publish(message);
      if(message.data == 150.0){
        message.data = 0.0;
      }
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_a;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_b;           
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<temperature_publisher>());
  rclcpp::shutdown();
  return 0;
}