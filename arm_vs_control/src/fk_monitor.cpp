#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>

#include "arm_vs_control/kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class FkMonitor : public rclcpp::Node
{
public:
  FkMonitor()
  : Node("fk_monitor")
  {
    // robot_state_publisher keeps the last robot_description message for late subscribers
    auto latched_qos = rclcpp::QoS(1).transient_local();
    description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", latched_qos,
      [this](const std_msgs::msg::String & msg) {on_description(msg);});

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_joint_states(msg);});

    timer_ = create_wall_timer(1s, [this]() {report();});
  }

private:
  void on_description(const std_msgs::msg::String & msg)
  {
    if (!kinematics_.init(msg.data, "panda_link0", "panda_hand")) {
      RCLCPP_ERROR(get_logger(), "Could not build the kinematic chain");
      return;
    }
    q_ = Eigen::VectorXd::Zero(kinematics_.num_joints());
    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "Kinematic chain ready with %u joints", kinematics_.num_joints());
  }

  void on_joint_states(const sensor_msgs::msg::JointState & msg)
  {
    if (!model_ready_) {
      return;
    }
    // joint_states may list joints in any order, so match them by name
    const auto & names = kinematics_.joint_names();
    for (size_t i = 0; i < names.size(); ++i) {
      for (size_t k = 0; k < msg.name.size(); ++k) {
        if (msg.name[k] == names[i]) {
          q_(i) = msg.position[k];
        }
      }
    }
    joints_received_ = true;
  }

  void report()
  {
    if (!model_ready_ || !joints_received_) {
      RCLCPP_INFO(get_logger(), "Waiting for robot model and joint states...");
      return;
    }

    Eigen::Isometry3d pose;
    Eigen::MatrixXd J;
    if (!kinematics_.forward(q_, pose) || !kinematics_.jacobian(q_, J)) {
      RCLCPP_ERROR(get_logger(), "Kinematics computation failed");
      return;
    }

    const Eigen::Vector3d p = pose.translation();
    const Eigen::Quaterniond rot(pose.linear());

    // How "comfortable" the robot is: close to 0 means near a singularity
    const double manipulability = std::sqrt((J * J.transpose()).determinant());

    std::stringstream ss;
    ss << J.format(Eigen::IOFormat(3, 0, ", ", "\n", "  [", "]"));

    RCLCPP_INFO(
      get_logger(),
      "\nHand position [m]:  x=%.3f  y=%.3f  z=%.3f"
      "\nHand rotation [quat]: x=%.3f  y=%.3f  z=%.3f  w=%.3f"
      "\nManipulability: %.4f"
      "\nJacobian (6x7):\n%s",
      p.x(), p.y(), p.z(), rot.x(), rot.y(), rot.z(), rot.w(),
      manipulability, ss.str().c_str());
  }

  arm_vs_control::Kinematics kinematics_;
  Eigen::VectorXd q_;
  bool model_ready_ = false;
  bool joints_received_ = false;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkMonitor>());
  rclcpp::shutdown();
  return 0;
}