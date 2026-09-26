// A deliberately small first C++/ROS exercise. Run separately from navigation.
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class WarmupPublisher : public rclcpp::Node {
 public:
  WarmupPublisher() : Node("warmup_publisher") {
    publisher_ = create_publisher<std_msgs::msg::String>("/test_topic", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      std_msgs::msg::String message;
      message.data = "Hello, ROS 2!";
      publisher_->publish(message);
      RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    });
  }
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WarmupPublisher>());
  rclcpp::shutdown();
}
