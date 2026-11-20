#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class VisualServo : public rclcpp::Node
{
public:
  VisualServo()
  : Node("visual_servo"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    standoff_ = declare_parameter("standoff", 0.20);             // camera height above marker [m]
    gain_ = declare_parameter("gain", 1.0);                      // lambda
    max_linear_ = declare_parameter("max_linear_speed", 0.08);   // [m/s]
    max_angular_ = declare_parameter("max_angular_speed", 0.4);  // [rad/s]

    // Schmitt trigger: stop below the first pair, start again above the second pair
    stop_position_ = declare_parameter("stop_position_error", 0.003);     // [m]
    stop_rotation_ = declare_parameter("stop_rotation_error", 0.017);     // [rad], 1 deg
    start_position_ = declare_parameter("start_position_error", 0.010);   // [m]
    start_rotation_ = declare_parameter("start_rotation_error", 0.052);   // [rad], 3 deg

    // Low-pass filter on the marker pose: 1.0 = no filtering, smaller = smoother
    filter_alpha_ = declare_parameter("filter_alpha", 0.3);

    // Outlier rejection: a jump bigger than this is ignored unless it repeats
    max_jump_ = declare_parameter("max_jump", 0.03);  // [m]
    outlier_limit_ = declare_parameter("outlier_limit", 5);

    // Safe workspace for the marker, in the robot base frame [m]
    ws_min_x_ = declare_parameter("workspace_min_x", 0.35);
    ws_max_x_ = declare_parameter("workspace_max_x", 0.65);
    ws_max_y_ = declare_parameter("workspace_max_abs_y", 0.25);

    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/marker_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {on_marker_pose(msg);});

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/target_twist", 10);

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {check_marker_timeout();});

    RCLCPP_INFO(get_logger(), "Visual servo ready, standoff %.2f m", standoff_);
  }

private:
  void on_marker_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    const std::string camera_frame = msg.header.frame_id;

    // 1. Read the transforms we need from TF
    geometry_msgs::msg::TransformStamped base_to_camera_msg;
    geometry_msgs::msg::TransformStamped camera_to_hand_msg;
    try {
      base_to_camera_msg = tf_buffer_.lookupTransform(
        "panda_link0", camera_frame, rclcpp::Time(msg.header.stamp),
        rclcpp::Duration::from_seconds(0.05));
      camera_to_hand_msg = tf_buffer_.lookupTransform(
        camera_frame, "panda_hand", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", ex.what());
      return;
    }

    const Eigen::Isometry3d T_base_camera = tf2::transformToEigen(base_to_camera_msg);
    const Eigen::Isometry3d T_camera_hand = tf2::transformToEigen(camera_to_hand_msg);
    Eigen::Isometry3d T_camera_marker;
    tf2::fromMsg(msg.pose, T_camera_marker);

    // Second line of defence against invalid numbers
    if (!T_camera_marker.matrix().allFinite()) {
      return;
    }

    last_marker_time_ = now();
    if (marker_lost_) {
      RCLCPP_INFO(get_logger(), "Marker found again");
      marker_lost_ = false;
    }

    // 2. Marker in the base frame, kept horizontal, then filtered
    const Eigen::Isometry3d raw_marker = keep_horizontal(T_base_camera * T_camera_marker);
    if (!update_filter(raw_marker)) {
      return;  // outlier, ignored
    }
    const Eigen::Isometry3d T_base_marker = filtered_marker();

    // Safety: do not follow a marker that is outside the reachable area
    const Eigen::Vector3d m = T_base_marker.translation();
    if (m.x() < ws_min_x_ || m.x() > ws_max_x_ || std::abs(m.y()) > ws_max_y_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Marker at x=%.2f y=%.2f is outside the workspace, holding position", m.x(), m.y());
      twist_pub_->publish(geometry_msgs::msg::Twist());
      holding_ = false;
      return;
    }

    // 3. Current hand pose (at the same moment as the image)
    const Eigen::Isometry3d T_base_hand = T_base_camera * T_camera_hand;

    // 4. Desired hand pose: pick the closest of the 4 symmetric cube orientations
    Eigen::Isometry3d best_target = Eigen::Isometry3d::Identity();
    double best_angle = 1e9;
    for (int k = 0; k < 4; ++k) {
      Eigen::Isometry3d T_marker_camera_goal = Eigen::Isometry3d::Identity();
      T_marker_camera_goal.translate(Eigen::Vector3d(0.0, 0.0, standoff_));
      T_marker_camera_goal.rotate(Eigen::AngleAxisd(k * M_PI / 2.0, Eigen::Vector3d::UnitZ()));
      T_marker_camera_goal.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

      const Eigen::Isometry3d candidate = T_base_marker * T_marker_camera_goal * T_camera_hand;
      const Eigen::AngleAxisd diff(candidate.linear() * T_base_hand.linear().transpose());
      if (diff.angle() < best_angle) {
        best_angle = diff.angle();
        best_target = candidate;
      }
    }

    // 5. Error between desired and current hand pose
    const Eigen::Vector3d position_error = best_target.translation() - T_base_hand.translation();
    const Eigen::AngleAxisd rotation_diff(best_target.linear() * T_base_hand.linear().transpose());
    const Eigen::Vector3d rotation_error = rotation_diff.angle() * rotation_diff.axis();
    const double pos_err = position_error.norm();
    const double rot_err = rotation_diff.angle();

    // 6. Schmitt trigger between "moving" and "holding"
    if (holding_) {
      if (pos_err > start_position_ || rot_err > start_rotation_) {
        holding_ = false;
        RCLCPP_INFO(get_logger(), "Marker moved, following it");
      }
    } else if (pos_err < stop_position_ && rot_err < stop_rotation_) {
      holding_ = true;
      RCLCPP_INFO(get_logger(), "Converged, holding position");
    }

    // 7. Control law, only while moving
    geometry_msgs::msg::Twist twist;  // all zeros by default
    if (!holding_) {
      Eigen::Vector3d v = gain_ * position_error;
      Eigen::Vector3d w = gain_ * rotation_error;
      if (v.norm() > max_linear_) {
        v *= max_linear_ / v.norm();
      }
      if (w.norm() > max_angular_) {
        w *= max_angular_ / w.norm();
      }
      twist.linear.x = v.x();
      twist.linear.y = v.y();
      twist.linear.z = v.z();
      twist.angular.x = w.x();
      twist.angular.y = w.y();
      twist.angular.z = w.z();
    }
    twist_pub_->publish(twist);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[%s] position error: %.1f mm, rotation error: %.1f deg",
      holding_ ? "HOLD" : "MOVE", pos_err * 1000.0, rot_err * 180.0 / M_PI);
  }

  // The cube lies on a flat table, so its top face is horizontal.
  // Keep only the rotation around the vertical axis and drop the noisy tilt.
  static Eigen::Isometry3d keep_horizontal(const Eigen::Isometry3d & T)
  {
    const Eigen::Vector3d x_axis = T.linear().col(0);
    const double yaw = std::atan2(x_axis.y(), x_axis.x());

    Eigen::Isometry3d flat = Eigen::Isometry3d::Identity();
    flat.translation() = T.translation();
    flat.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return flat;
  }

  // Returns false if the measurement was rejected as an outlier
  bool update_filter(const Eigen::Isometry3d & raw)
  {
    const Eigen::Vector3d raw_position = raw.translation();
    Eigen::Quaterniond raw_rotation(raw.linear());

    if (!filter_initialized_) {
      filtered_position_ = raw_position;
      filtered_rotation_ = raw_rotation;
      filter_initialized_ = true;
      outlier_count_ = 0;
      return true;
    }

    // Outlier rejection: ignore a sudden jump, unless it keeps repeating
    const double jump = (raw_position - filtered_position_).norm();
    if (jump > max_jump_) {
      ++outlier_count_;
      if (outlier_count_ < outlier_limit_) {
        RCLCPP_WARN(get_logger(), "Ignoring outlier: marker jumped %.1f mm", jump * 1000.0);
        return false;
      }
      // The jump is consistent: the cube really moved, start the filter again there
      RCLCPP_INFO(get_logger(), "Marker position changed a lot, accepting new position");
      filtered_position_ = raw_position;
      filtered_rotation_ = raw_rotation;
      outlier_count_ = 0;
      return true;
    }
    outlier_count_ = 0;

    // Position: exponential moving average (like an RC low-pass filter)
    filtered_position_ = filter_alpha_ * raw_position + (1.0 - filter_alpha_) * filtered_position_;

    // Rotation: move a fraction of the way towards the new measurement
    if (filtered_rotation_.dot(raw_rotation) < 0.0) {
      raw_rotation.coeffs() *= -1.0;  // q and -q are the same rotation; pick the closer one
    }
    filtered_rotation_ = filtered_rotation_.slerp(filter_alpha_, raw_rotation).normalized();
    return true;
  }

  Eigen::Isometry3d filtered_marker() const
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = filtered_position_;
    T.linear() = filtered_rotation_.toRotationMatrix();
    return T;
  }

  void check_marker_timeout()
  {
    if (last_marker_time_.nanoseconds() == 0) {
      return;  // marker never seen yet
    }
    if (!marker_lost_ && (now() - last_marker_time_).seconds() > 0.5) {
      RCLCPP_WARN(get_logger(), "Marker lost, robot stops until it is visible again");
      marker_lost_ = true;
      filter_initialized_ = false;  // start fresh when the marker comes back
      holding_ = false;
    }
  }

  // Settings
  double standoff_;
  double gain_;
  double max_linear_;
  double max_angular_;
  double stop_position_;
  double stop_rotation_;
  double start_position_;
  double start_rotation_;
  double filter_alpha_;
  double max_jump_;
  int outlier_limit_;
  double ws_min_x_;
  double ws_max_x_;
  double ws_max_y_;

  // State
  bool holding_ = false;
  bool marker_lost_ = false;
  bool filter_initialized_ = false;
  int outlier_count_ = 0;
  Eigen::Vector3d filtered_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond filtered_rotation_ = Eigen::Quaterniond::Identity();
  rclcpp::Time last_marker_time_{0, 0, RCL_ROS_TIME};

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualServo>());
  rclcpp::shutdown();
  return 0;
}