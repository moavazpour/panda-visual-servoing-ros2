#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class AngleListener : public rclcpp::Node
{
public:
  AngleListener()
  : Node("angle_listener")
  {
    subscription_ = create_subscription<std_msgs::msg::Float64>(
      "joint_angle", 10,
      [this](const std_msgs::msg::Float64 & msg) { on_angle(msg); });
  }

private:
  void on_angle(const std_msgs::msg::Float64 & msg)
  {
    RCLCPP_INFO(get_logger(), "received %.1f deg", msg.data * 180.0 / M_PI);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleListener>());
  rclcpp::shutdown();
  return 0;
}