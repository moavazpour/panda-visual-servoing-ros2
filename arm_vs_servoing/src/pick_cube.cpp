#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class PickCube : public rclcpp::Node
{
public:
  PickCube()
  : Node("pick_cube"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    tcp_offset_ = declare_parameter("tcp_offset", 0.1034);       // hand origin -> fingertip centre [m]
    cube_size_ = declare_parameter("cube_size", 0.05);           // [m]
    // Grasp this much above the estimated cube centre, as a margin for depth error [m]
    grasp_height_offset_ = declare_parameter("grasp_height_offset", 0.015);
    pregrasp_height_ = declare_parameter("pregrasp_height", 0.10);  // above the grasp pose [m]
    lift_height_ = declare_parameter("lift_height", 0.15);       // above the grasp pose [m]
    gain_ = declare_parameter("gain", 1.5);
    fast_speed_ = declare_parameter("fast_speed", 0.08);         // [m/s]
    slow_speed_ = declare_parameter("slow_speed", 0.03);         // [m/s], near the cube
    max_angular_ = declare_parameter("max_angular_speed", 0.4);  // [rad/s]
    open_width_ = declare_parameter("open_width", 0.04);         // finger position when open [m]

    cube_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cube_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {
        tf2::fromMsg(msg.pose, cube_top_);
        has_cube_ = true;
      });

    holding_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/servo_holding", 10,
      [this](const std_msgs::msg::Bool & msg) {servo_holding_ = msg.data;});

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState & msg) {
        for (size_t i = 0; i < msg.name.size(); ++i) {
          if (msg.name[i] == "panda_finger_joint1") {
            finger_position_ = msg.position[i];
          }
        }
      });

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/target_twist", 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_controller/commands", 10);
    servo_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/servo_enable", 10);

    timer_ = create_wall_timer(20ms, [this]() {step();});
    state_start_ = now();
    last_motion_time_ = now();

    RCLCPP_INFO(get_logger(), "Pick node ready, waiting for the servo to settle above the cube");
  }

private:
  enum class State
  {
    WAIT_FOR_CUBE,
    OPEN_GRIPPER,
    PRE_GRASP,
    DESCEND,
    CLOSE_GRIPPER,
    LIFT,
    DONE,
    FAILED
  };

  // ---------------- State machine ----------------

  void step()
  {
    switch (state_) {
      case State::WAIT_FOR_CUBE:
        if (servo_holding_ && has_cube_) {
          if (seconds_in_state() > 1.0) {
            plan_grasp();
            disable_servo();
            set_state(State::OPEN_GRIPPER);
          }
        } else {
          state_start_ = now();
        }
        break;

      case State::OPEN_GRIPPER:
        command_gripper(open_width_);
        if (seconds_in_state() > 1.0) {
          set_state(State::PRE_GRASP);
        }
        break;

      case State::PRE_GRASP:
        if (go_to(pregrasp_pose_, fast_speed_)) {
          set_state(State::DESCEND);
        }
        break;

      case State::DESCEND:
        if (go_to(grasp_pose_, slow_speed_)) {
          set_state(State::CLOSE_GRIPPER);
        } else if (contact_detected()) {
          RCLCPP_WARN(
            get_logger(), "Hand stopped %.1f mm above the target (contact), closing here",
            (current_hand_pose().translation().z() - grasp_pose_.translation().z()) * 1000.0);
          set_state(State::CLOSE_GRIPPER);
        }
        break;

      case State::CLOSE_GRIPPER:
        stop();
        command_gripper(0.0);
        if (seconds_in_state() > 1.5) {
          if (finger_position_ < 0.005) {
            RCLCPP_ERROR(get_logger(), "Fingers closed on nothing, the cube was missed");
            set_state(State::FAILED);
          } else {
            RCLCPP_INFO(
              get_logger(), "Cube between the fingers, opening %.1f mm",
              2.0 * finger_position_ * 1000.0);
            set_state(State::LIFT);
          }
        }
        break;

      case State::LIFT:
        if (go_to(lift_pose_, slow_speed_)) {
          set_state(State::DONE);
        }
        break;

      case State::DONE:
      case State::FAILED:
        stop();
        break;
    }
  }

  void set_state(State next)
  {
    static const char * names[] = {
      "WAIT_FOR_CUBE", "OPEN_GRIPPER", "PRE_GRASP", "DESCEND",
      "CLOSE_GRIPPER", "LIFT", "DONE", "FAILED"};
    RCLCPP_INFO(
      get_logger(), "%s -> %s",
      names[static_cast<int>(state_)], names[static_cast<int>(next)]);
    state_ = next;
    state_start_ = now();
    last_motion_time_ = now();
    last_hand_z_ = 1e9;
  }

  double seconds_in_state() {return (now() - state_start_).seconds();}

  // True if the hand has not moved for 0.5 s while it is close to the grasp pose
  bool contact_detected()
  {
    const double z = current_hand_pose().translation().z();
    if (std::abs(z - last_hand_z_) > 0.0005) {
      last_hand_z_ = z;
      last_motion_time_ = now();
      return false;
    }
    const double remaining = z - grasp_pose_.translation().z();
    return remaining < 0.03 && (now() - last_motion_time_).seconds() > 0.5;
  }

  // ---------------- Planning ----------------

  void plan_grasp()
  {
    const Eigen::Isometry3d hand_now = current_hand_pose();

    const Eigen::Vector3d cube_centre =
      cube_top_.translation() - Eigen::Vector3d(0.0, 0.0, cube_size_ / 2.0);
    const double cube_yaw = std::atan2(cube_top_.linear()(1, 0), cube_top_.linear()(0, 0));

    double best_angle = 1e9;
    for (int k = 0; k < 4; ++k) {
      const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(cube_yaw + k * M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
      const double angle = Eigen::AngleAxisd(R * hand_now.linear().transpose()).angle();
      if (angle < best_angle) {
        best_angle = angle;
        grasp_pose_.linear() = R;
      }
    }

    // Fingertip centre a bit above the cube centre -> hand origin tcp_offset higher
    grasp_pose_.translation() =
      cube_centre + Eigen::Vector3d(0.0, 0.0, grasp_height_offset_ + tcp_offset_);

    pregrasp_pose_ = grasp_pose_;
    pregrasp_pose_.translation().z() += pregrasp_height_;

    lift_pose_ = grasp_pose_;
    lift_pose_.translation().z() += lift_height_;

    RCLCPP_INFO(
      get_logger(), "Cube centre at x=%.3f y=%.3f z=%.3f, yaw %.1f deg",
      cube_centre.x(), cube_centre.y(), cube_centre.z(), cube_yaw * 180.0 / M_PI);
  }

  // ---------------- Motion ----------------

  bool go_to(const Eigen::Isometry3d & target, double max_speed)
  {
    const Eigen::Isometry3d hand = current_hand_pose();

    const Eigen::Vector3d position_error = target.translation() - hand.translation();
    const Eigen::AngleAxisd rotation_diff(target.linear() * hand.linear().transpose());
    const Eigen::Vector3d rotation_error = rotation_diff.angle() * rotation_diff.axis();

    if (position_error.norm() < 0.002 && rotation_diff.angle() < 0.017) {
      stop();
      return true;
    }

    Eigen::Vector3d v = gain_ * position_error;
    Eigen::Vector3d w = gain_ * rotation_error;
    if (v.norm() > max_speed) {
      v *= max_speed / v.norm();
    }
    if (w.norm() > max_angular_) {
      w *= max_angular_ / w.norm();
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = v.x();
    twist.linear.y = v.y();
    twist.linear.z = v.z();
    twist.angular.x = w.x();
    twist.angular.y = w.y();
    twist.angular.z = w.z();
    twist_pub_->publish(twist);
    return false;
  }

  void stop() {twist_pub_->publish(geometry_msgs::msg::Twist());}

  Eigen::Isometry3d current_hand_pose()
  {
    try {
      return tf2::transformToEigen(
        tf_buffer_.lookupTransform("panda_link0", "panda_hand", tf2::TimePointZero));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", ex.what());
      return Eigen::Isometry3d::Identity();
    }
  }

  void command_gripper(double finger_position)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {finger_position};
    gripper_pub_->publish(msg);
  }

  void disable_servo()
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    servo_enable_pub_->publish(msg);
  }

  // Settings
  double tcp_offset_;
  double cube_size_;
  double grasp_height_offset_;
  double pregrasp_height_;
  double lift_height_;
  double gain_;
  double fast_speed_;
  double slow_speed_;
  double max_angular_;
  double open_width_;

  // State
  State state_ = State::WAIT_FOR_CUBE;
  rclcpp::Time state_start_;
  rclcpp::Time last_motion_time_;
  double last_hand_z_ = 1e9;
  bool has_cube_ = false;
  bool servo_holding_ = false;
  double finger_position_ = 0.0;
  Eigen::Isometry3d cube_top_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d grasp_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pregrasp_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lift_pose_ = Eigen::Isometry3d::Identity();

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr holding_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickCube>());
  rclcpp::shutdown();
  return 0;
}