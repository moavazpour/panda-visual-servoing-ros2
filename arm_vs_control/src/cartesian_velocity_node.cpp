#include <chrono>
#include <cmath>
#include <memory>

#include "arm_vs_control/kinematics.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CartesianVelocityNode : public rclcpp::Node
{
public:
  CartesianVelocityNode()
  : Node("cartesian_velocity_node")
  {
    max_damping_ = declare_parameter("max_damping", 0.05);
    manipulability_threshold_ = declare_parameter("manipulability_threshold", 0.03);
    position_gain_ = declare_parameter("position_gain", 2.0);
    max_joint_velocity_ = declare_parameter("max_joint_velocity", 0.5);
    nullspace_gain_ = declare_parameter("nullspace_gain", 0.5);
    command_timeout_ = declare_parameter("command_timeout", 0.2);

    // Anti-windup: how far the internal target may run ahead of the real hand [m]
    max_position_lead_ = declare_parameter("max_position_lead", 0.01);

    // Null space, 2nd task: preferred posture. The elbow swivel moves joint 3 away from home,
    // and this weight makes joint 3 matter more than the others.
    swivel_weight_ = declare_parameter("swivel_weight", 4.0);

    // Null space, safety: keep the elbow above the table, the cube and the shelf
    elbow_min_height_ = declare_parameter("elbow_min_height", 0.35);  // [m]
    elbow_gain_ = declare_parameter("elbow_gain", 20.0);

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

    swivel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/elbow_swivel", 10,
      [this](const std_msgs::msg::Float64 & msg) {swivel_ = msg.data;});

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/velocity_controller/commands", 10);

    timer_ = create_wall_timer(10ms, [this]() {control_step();});
  }

private:
  static constexpr double kDt = 0.01;  // control period [s], matches the 10 ms timer

  void on_description(const std_msgs::msg::String & msg)
  {
    if (!kinematics_.init(msg.data, "panda_link0", "panda_hand")) {
      RCLCPP_ERROR(get_logger(), "Could not build the kinematic chain");
      return;
    }
    // Second chain, base to elbow, to know where the elbow is
    if (!elbow_kinematics_.init(msg.data, "panda_link0", "panda_link4")) {
      RCLCPP_ERROR(get_logger(), "Could not build the elbow chain");
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

    const bool command_fresh =
      has_command_ && (now() - last_command_time_).seconds() < command_timeout_;

    Eigen::Isometry3d current_pose;
    Eigen::MatrixXd J;
    if (!kinematics_.forward(q_, current_pose) || !kinematics_.jacobian(q_, J)) {
      return;
    }

    if (!command_fresh) {
      // Idle: forget the target, it will be set again when a new command starts
      tracking_ = false;
      publish(qdot);
      return;
    }

    // 1. When a new command starts, the current hand pose becomes the target
    if (!tracking_) {
      target_pose_ = current_pose;
      tracking_ = true;
    }

    // 2. Move the target forward with the commanded velocity
    const Eigen::Vector3d v = twist_.head<3>();
    const Eigen::Vector3d w = twist_.tail<3>();
    target_pose_.translation() += v * kDt;
    if (w.norm() > 1e-9) {
      const Eigen::AngleAxisd step(w.norm() * kDt, w.normalized());
      target_pose_.linear() = step.toRotationMatrix() * target_pose_.linear();
    }
    
    // Anti-windup: if the hand is blocked, for example by a surface it touches,
    // keep the target close to the real hand, so the pushing force cannot keep growing
    const Eigen::Vector3d lead = target_pose_.translation() - current_pose.translation();
    if (lead.norm() > max_position_lead_) {
      target_pose_.translation() =
        current_pose.translation() + lead.normalized() * max_position_lead_;
    }

    // 3. Error between target and real pose (position and orientation)
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target_pose_.translation() - current_pose.translation();
    const Eigen::AngleAxisd rot_error(target_pose_.linear() * current_pose.linear().transpose());
    error.tail<3>() = rot_error.angle() * rot_error.axis();

    // 4. Closed loop: commanded velocity + correction proportional to the error
    const Eigen::Matrix<double, 6, 1> twist_total = twist_ + position_gain_ * error;

    // 5. Damping only near a singularity
    const Eigen::MatrixXd JJt = J * J.transpose();
    const double manipulability = std::sqrt(JJt.determinant());
    double damping = 0.0;
    if (manipulability < manipulability_threshold_) {
      const double ratio = manipulability / manipulability_threshold_;
      damping = max_damping_ * std::sqrt(1.0 - ratio * ratio);
    }
    const Eigen::MatrixXd J_pinv_damped =
      J.transpose() * (JJt + damping * damping * Eigen::MatrixXd::Identity(6, 6)).inverse();

    // 6. Main task
    qdot = J_pinv_damped * twist_total;

    // 7. Secondary task with an exact null-space projector
    const Eigen::MatrixXd J_pinv_exact = J.completeOrthogonalDecomposition().pseudoInverse();
    const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J_pinv_exact * J;
    qdot += N * nullspace_velocity();

    // 8. Speed limit, all joints scaled together
    const double fastest = qdot.cwiseAbs().maxCoeff();
    if (fastest > max_joint_velocity_) {
      qdot *= max_joint_velocity_ / fastest;
    }

    publish(qdot);
  }

  // What the arm should do with its spare degree of freedom.
  // The projector N removes everything that would move the hand.
  Eigen::VectorXd nullspace_velocity()
  {
    const auto n = kinematics_.num_joints();

    // a) Preferred posture: home, with joint 3 turned by the requested swivel
    Eigen::VectorXd q_preferred = q_home_;
    q_preferred(2) += swivel_;
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(n);
    weights(2) = swivel_weight_;
    Eigen::VectorXd velocity = nullspace_gain_ * weights.cwiseProduct(q_preferred - q_);

    // b) Safety: if the elbow is too low, push it up.
    //    The elbow chain has only the first 4 joints; the others do not move the elbow.
    const auto m = elbow_kinematics_.num_joints();
    Eigen::Isometry3d elbow;
    Eigen::MatrixXd J_elbow;
    if (elbow_kinematics_.forward(q_.head(m), elbow) &&
      elbow_kinematics_.jacobian(q_.head(m), J_elbow))
    {
      const double too_low = elbow_min_height_ - elbow.translation().z();
      if (too_low > 0.0) {
        // Row 2 of the Jacobian: how each joint moves the elbow up or down
        velocity.head(m) += elbow_gain_ * too_low * J_elbow.row(2).transpose();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Elbow low (%.2f m), lifting it",
          elbow.translation().z());
      }
    }
    return velocity;
  }

  void publish(const Eigen::VectorXd & qdot)
  {
    std_msgs::msg::Float64MultiArray command;
    command.data.assign(qdot.data(), qdot.data() + qdot.size());
    command_pub_->publish(command);
  }

  // Settings
  double max_damping_;
  double manipulability_threshold_;
  double position_gain_;
  double max_joint_velocity_;
  double nullspace_gain_;
  double command_timeout_;
  double max_position_lead_;
  double swivel_weight_;
  double elbow_min_height_;
  double elbow_gain_;
  
  // State
  arm_vs_control::Kinematics kinematics_;
  arm_vs_control::Kinematics elbow_kinematics_;
  double swivel_ = 0.0;
  Eigen::VectorXd q_;
  Eigen::VectorXd q_home_;
  Eigen::Matrix<double, 6, 1> twist_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Isometry3d target_pose_ = Eigen::Isometry3d::Identity();
  rclcpp::Time last_command_time_;
  bool model_ready_ = false;
  bool joints_received_ = false;
  bool has_command_ = false;
  bool tracking_ = false;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr swivel_sub_;
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