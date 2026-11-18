#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class FakeJointPublisher : public rclcpp::Node
{
public:
  FakeJointPublisher()
  : Node("fake_joint_publisher")
  {
    publisher_ = create_publisher<std_msgs::msg::Float64>("joint_angle", 10);
    timer_ = create_wall_timer(100ms, [this]() { on_timer(); });
    start_time_ = now();
  }

private:
  void on_timer()
  {
    const double t = (now() - start_time_).seconds();
    std_msgs::msg::Float64 msg;
    msg.data = std::sin(t);
    publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "published angle = %.3f rad", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeJointPublisher>());
  rclcpp::shutdown();
  return 0;
}