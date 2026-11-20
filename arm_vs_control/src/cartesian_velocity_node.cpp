#include <chrono>
#include <memory>

#include "arm_vs_control/kinematics.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CartesianVelocityNode : public rclcpp::Node
{
public:
  CartesianVelocityNode()
  : Node("cartesian_velocity_node")
  {
    // Tunable settings, can be changed at startup with --ros-args -p name:=value
    damping_ = declare_parameter("damping", 0.05);
    max_joint_velocity_ = declare_parameter("max_joint_velocity", 0.5);
    nullspace_gain_ = declare_parameter("nullspace_gain", 0.5);
    command_timeout_ = declare_parameter("command_timeout", 0.2);

    auto latched_qos = rclcpp::QoS(1).transient_local();
    description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", latched_qos,
      [this](const std_msgs::msg::String & msg) {on_description(msg);});

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_joint_states(msg);});

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/target_twist", 10,
      [this](const geometry_msgs::msg::Twist & msg) {on_twist(msg);});

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/velocity_controller/commands", 10);

    // Control loop at 100 Hz
    timer_ = create_wall_timer(10ms, [this]() {control_step();});
  }

private:
  void on_description(const std_msgs::msg::String & msg)
  {
    if (!kinematics_.init(msg.data, "panda_link0", "panda_hand")) {
      RCLCPP_ERROR(get_logger(), "Could not build the kinematic chain");
      return;
    }
    const auto n = kinematics_.num_joints();
    q_ = Eigen::VectorXd::Zero(n);
    q_home_ = Eigen::VectorXd(n);
    q_home_ << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "Ready. Send geometry_msgs/Twist on /target_twist");
  }

  void on_joint_states(const sensor_msgs::msg::JointState & msg)
  {
    if (!model_ready_) {
      return;
    }
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

  void on_twist(const geometry_msgs::msg::Twist & msg)
  {
    // Order: 3 linear [m/s], then 3 angular [rad/s], all in the base frame
    twist_ << msg.linear.x, msg.linear.y, msg.linear.z,
      msg.angular.x, msg.angular.y, msg.angular.z;
    last_command_time_ = now();
    has_command_ = true;
  }

  void control_step()
  {
    if (!model_ready_ || !joints_received_) {
      return;
    }

    const auto n = kinematics_.num_joints();
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(n);

    // Safety: only move if the last command is recent (watchdog)
    const bool command_fresh =
      has_command_ && (now() - last_command_time_).seconds() < command_timeout_;

    if (command_fresh) {
      Eigen::MatrixXd J;
      if (!kinematics_.jacobian(q_, J)) {
        return;
      }

      // 1. Damped least-squares pseudo-inverse
      const Eigen::MatrixXd JJt =
        J * J.transpose() + damping_ * damping_ * Eigen::MatrixXd::Identity(6, 6);
      const Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();

      // 2. Main task: follow the requested hand velocity
      qdot = J_pinv * twist_;

      // 3. Secondary task in the null space: pull joints towards the home pose
      const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J_pinv * J;
      qdot += N * (nullspace_gain_ * (q_home_ - q_));

      // 4. Safety: limit joint speed, scale all joints together to keep the direction
      const double fastest = qdot.cwiseAbs().maxCoeff();
      if (fastest > max_joint_velocity_) {
        qdot *= max_joint_velocity_ / fastest;
      }
    }

    std_msgs::msg::Float64MultiArray command;
    command.data.assign(qdot.data(), qdot.data() + qdot.size());
    command_pub_->publish(command);
  }

  // Settings
  double damping_;
  double max_joint_velocity_;
  double nullspace_gain_;
  double command_timeout_;

  // State
  arm_vs_control::Kinematics kinematics_;
  Eigen::VectorXd q_;
  Eigen::VectorXd q_home_;
  Eigen::Matrix<double, 6, 1> twist_ = Eigen::Matrix<double, 6, 1>::Zero();
  rclcpp::Time last_command_time_;
  bool model_ready_ = false;
  bool joints_received_ = false;
  bool has_command_ = false;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianVelocityNode>());
  rclcpp::shutdown();
  return 0;
}