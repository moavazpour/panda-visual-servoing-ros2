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

// Averages several horizontal marker poses (position and yaw)
struct PoseAverager
{
  Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();
  double cos_sum = 0.0;
  double sin_sum = 0.0;
  int count = 0;

  void reset()
  {
    position_sum.setZero();
    cos_sum = 0.0;
    sin_sum = 0.0;
    count = 0;
  }

  void add(const Eigen::Isometry3d & T)
  {
    position_sum += T.translation();
    const double yaw = std::atan2(T.linear()(1, 0), T.linear()(0, 0));
    cos_sum += std::cos(yaw);
    sin_sum += std::sin(yaw);
    ++count;
  }

  Eigen::Isometry3d mean() const
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = position_sum / count;
    T.linear() = Eigen::AngleAxisd(
      std::atan2(sin_sum, cos_sum), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return T;
  }
};

class PickAndPlace : public rclcpp::Node
{
public:
  PickAndPlace()
  : Node("pick_and_place"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    tcp_offset_ = declare_parameter("tcp_offset", 0.1034);       // hand origin -> fingertip centre [m]
    cube_size_ = declare_parameter("cube_size", 0.05);           // [m]
    grasp_height_offset_ = declare_parameter("grasp_height_offset", 0.015);  // [m]
    pregrasp_height_ = declare_parameter("pregrasp_height", 0.10);  // [m]
    lift_height_ = declare_parameter("lift_height", 0.15);       // [m]
    standoff_ = declare_parameter("standoff", 0.20);             // camera above place marker [m]
    place_approach_height_ = declare_parameter("place_approach_height", 0.10);  // [m]
    place_push_ = declare_parameter("place_push", 0.01);         // aim this much below contact [m]
    retreat_height_ = declare_parameter("retreat_height", 0.15);  // [m]
    gain_ = declare_parameter("gain", 1.5);
    fast_speed_ = declare_parameter("fast_speed", 0.08);         // [m/s]
    slow_speed_ = declare_parameter("slow_speed", 0.03);         // [m/s]
    max_angular_ = declare_parameter("max_angular_speed", 0.4);  // [rad/s]
    open_width_ = declare_parameter("open_width", 0.04);         // [m]
    observe_time_ = declare_parameter("observe_time", 1.0);      // [s]
    min_samples_ = declare_parameter("min_samples", 10);

    // Overview pose: the same hand pose as the robot's start position
    scan_pose_ = Eigen::Isometry3d::Identity();
    scan_pose_.translation() = Eigen::Vector3d(0.307, 0.0, 0.59);
    scan_pose_.linear() =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

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

    place_marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/place_marker_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {
        Eigen::Isometry3d T;
        if (collecting_place_ && marker_in_base(msg, T)) {
          place_samples_.add(T);
        }
      });

    cube_marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/marker_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {
        Eigen::Isometry3d T;
        if (collecting_verify_ && marker_in_base(msg, T)) {
          verify_samples_.add(T);
        }
      });

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/target_twist", 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_controller/commands", 10);
    servo_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/servo_enable", 10);

    timer_ = create_wall_timer(20ms, [this]() {step();});
    state_start_ = now();
    last_motion_time_ = now();

    RCLCPP_INFO(get_logger(), "Pick and place ready, waiting for the servo above the cube");
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
    GO_TO_SCAN,
    LOOK_FOR_PLACE,
    GO_ABOVE_PLACE,
    MEASURE_PLACE,
    PRE_PLACE,
    PLACE_DESCEND,
    RELEASE,
    RETREAT,
    VERIFY,
    DONE,
    FAILED
  };

  // ======================= State machine =======================

  void step()
  {
    switch (state_) {
      // ---------- Pick ----------
      case State::WAIT_FOR_CUBE:
       if (servo_holding_ && has_cube_ &&
          tf_buffer_.canTransform("panda_link0", "panda_hand", tf2::TimePointZero))
        {
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
        if (go_to(grasp_pose_, slow_speed_) || contact_detected(grasp_pose_)) {
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
          set_state(State::GO_TO_SCAN);
        }
        break;

      // ---------- Find the place location: coarse ----------
      case State::GO_TO_SCAN:
        if (go_to(scan_pose_, fast_speed_)) {
          place_samples_.reset();
          collecting_place_ = true;
          set_state(State::LOOK_FOR_PLACE);
        }
        break;

      case State::LOOK_FOR_PLACE:
        stop();
        if (seconds_in_state() > observe_time_ && place_samples_.count >= min_samples_) {
          collecting_place_ = false;
          place_estimate_ = place_samples_.mean();
          observe_pose_ = camera_above(place_estimate_);
          log_place("Coarse place estimate");
          set_state(State::GO_ABOVE_PLACE);
        } else if (seconds_in_state() > 5.0) {
          RCLCPP_ERROR(get_logger(), "Place marker not visible from the scan pose");
          set_state(State::FAILED);
        }
        break;

      // ---------- Find the place location: fine ----------
      case State::GO_ABOVE_PLACE:
        if (go_to(observe_pose_, fast_speed_)) {
          place_samples_.reset();
          collecting_place_ = true;
          set_state(State::MEASURE_PLACE);
        }
        break;

      case State::MEASURE_PLACE:
        stop();
        if (seconds_in_state() > observe_time_ && place_samples_.count >= min_samples_) {
          collecting_place_ = false;
          place_estimate_ = place_samples_.mean();
          log_place("Fine place estimate");
          plan_place();
          set_state(State::PRE_PLACE);
        } else if (seconds_in_state() > 5.0) {
          RCLCPP_ERROR(get_logger(), "Place marker lost when looking from close by");
          set_state(State::FAILED);
        }
        break;

      // ---------- Place ----------
      case State::PRE_PLACE:
        if (go_to(preplace_pose_, fast_speed_)) {
          set_state(State::PLACE_DESCEND);
        }
        break;

      case State::PLACE_DESCEND:
        if (go_to(place_pose_, slow_speed_) || contact_detected(place_pose_)) {
          set_state(State::RELEASE);
        }
        break;

      case State::RELEASE:
        stop();
        command_gripper(open_width_);
        if (seconds_in_state() > 1.0) {
          set_state(State::RETREAT);
        }
        break;

      case State::RETREAT:
        if (go_to(retreat_pose_, slow_speed_)) {
          verify_samples_.reset();
          collecting_verify_ = true;
          set_state(State::VERIFY);
        }
        break;

      // ---------- Measure how well we did ----------
      case State::VERIFY:
        stop();
        if (seconds_in_state() > observe_time_ && verify_samples_.count >= min_samples_) {
          collecting_verify_ = false;
          report_placement_error();
          set_state(State::DONE);
        } else if (seconds_in_state() > 4.0) {
          collecting_verify_ = false;
          RCLCPP_WARN(get_logger(), "Cube marker not visible, cannot verify the placement");
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
      "WAIT_FOR_CUBE", "OPEN_GRIPPER", "PRE_GRASP", "DESCEND", "CLOSE_GRIPPER", "LIFT",
      "GO_TO_SCAN", "LOOK_FOR_PLACE", "GO_ABOVE_PLACE", "MEASURE_PLACE",
      "PRE_PLACE", "PLACE_DESCEND", "RELEASE", "RETREAT", "VERIFY", "DONE", "FAILED"};
    RCLCPP_INFO(
      get_logger(), "%s -> %s",
      names[static_cast<int>(state_)], names[static_cast<int>(next)]);
    state_ = next;
    state_start_ = now();
    last_motion_time_ = now();
    last_hand_z_ = 1e9;
  }

  double seconds_in_state() {return (now() - state_start_).seconds();}

  // True if the hand has not moved for 0.5 s while it is close to the target
  bool contact_detected(const Eigen::Isometry3d & target)
  {
    const double z = current_hand_pose().translation().z();
    if (std::abs(z - last_hand_z_) > 0.0005) {
      last_hand_z_ = z;
      last_motion_time_ = now();
      return false;
    }
    const double remaining = z - target.translation().z();
    if (remaining < 0.03 && (now() - last_motion_time_).seconds() > 0.5) {
      RCLCPP_WARN(
        get_logger(), "Contact detected %.1f mm above the target", remaining * 1000.0);
      return true;
    }
    return false;
  }

  // ======================= Planning =======================

  void plan_grasp()
  {
    const Eigen::Vector3d cube_centre =
      cube_top_.translation() - Eigen::Vector3d(0.0, 0.0, cube_size_ / 2.0);

    grasp_pose_.linear() = aligned_hand_rotation(yaw_of(cube_top_));
    grasp_pose_.translation() =
      cube_centre + Eigen::Vector3d(0.0, 0.0, grasp_height_offset_ + tcp_offset_);

    pregrasp_pose_ = grasp_pose_;
    pregrasp_pose_.translation().z() += pregrasp_height_;

    lift_pose_ = grasp_pose_;
    lift_pose_.translation().z() += lift_height_;

    RCLCPP_INFO(
      get_logger(), "Cube centre at x=%.3f y=%.3f z=%.3f, yaw %.1f deg",
      cube_centre.x(), cube_centre.y(), cube_centre.z(), yaw_of(cube_top_) * 180.0 / M_PI);
  }

  void plan_place()
  {
    const Eigen::Vector3d p = place_estimate_.translation();

    // Hand height at which the cube bottom touches the shelf
    const double contact_z = p.z() + cube_size_ / 2.0 + grasp_height_offset_ + tcp_offset_;

    place_pose_ = Eigen::Isometry3d::Identity();
    place_pose_.linear() = aligned_hand_rotation(yaw_of(place_estimate_));
    // Aim a little below contact, so the contact detection stops the hand
    place_pose_.translation() = Eigen::Vector3d(p.x(), p.y(), contact_z - place_push_);

    preplace_pose_ = place_pose_;
    preplace_pose_.translation().z() = contact_z + place_approach_height_;

    retreat_pose_ = place_pose_;
    retreat_pose_.translation().z() = contact_z + retreat_height_;
  }

  // Hand pointing down, fingers parallel to a face of a square object with this yaw.
  // A square looks the same every 90 degrees: pick the option closest to the hand now.
  Eigen::Matrix3d aligned_hand_rotation(double yaw)
  {
    const Eigen::Isometry3d hand_now = current_hand_pose();
    Eigen::Matrix3d best = Eigen::Matrix3d::Identity();
    double best_angle = 1e9;
    for (int k = 0; k < 4; ++k) {
      const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(yaw + k * M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
      const double angle = Eigen::AngleAxisd(R * hand_now.linear().transpose()).angle();
      if (angle < best_angle) {
        best_angle = angle;
        best = R;
      }
    }
    return best;
  }

  // Hand pose that puts the camera 'standoff' above a marker, looking straight down
  Eigen::Isometry3d camera_above(const Eigen::Isometry3d & marker)
  {
    Eigen::Isometry3d T_camera_hand = Eigen::Isometry3d::Identity();
    try {
      T_camera_hand = tf2::transformToEigen(
        tf_buffer_.lookupTransform(camera_frame_, "panda_hand", tf2::TimePointZero));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF not ready: %s", ex.what());
    }

    const Eigen::Isometry3d hand_now = current_hand_pose();
    Eigen::Isometry3d best = hand_now;
    double best_angle = 1e9;
    for (int k = 0; k < 4; ++k) {
      Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
      goal.translate(Eigen::Vector3d(0.0, 0.0, standoff_));
      goal.rotate(Eigen::AngleAxisd(k * M_PI / 2.0, Eigen::Vector3d::UnitZ()));
      goal.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

      const Eigen::Isometry3d candidate = marker * goal * T_camera_hand;
      const double angle =
        Eigen::AngleAxisd(candidate.linear() * hand_now.linear().transpose()).angle();
      if (angle < best_angle) {
        best_angle = angle;
        best = candidate;
      }
    }
    return best;
  }

  // ======================= Perception helpers =======================

  // Marker pose (camera frame) -> horizontal marker pose in the robot base frame
  bool marker_in_base(const geometry_msgs::msg::PoseStamped & msg, Eigen::Isometry3d & out)
  {
    camera_frame_ = msg.header.frame_id;
    Eigen::Isometry3d T_camera_marker;
    tf2::fromMsg(msg.pose, T_camera_marker);
    if (!T_camera_marker.matrix().allFinite()) {
      return false;
    }
    try {
      const Eigen::Isometry3d T_base_camera = tf2::transformToEigen(
        tf_buffer_.lookupTransform(
          "panda_link0", msg.header.frame_id, rclcpp::Time(msg.header.stamp),
          rclcpp::Duration::from_seconds(0.05)));
      out = keep_horizontal(T_base_camera * T_camera_marker);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static Eigen::Isometry3d keep_horizontal(const Eigen::Isometry3d & T)
  {
    Eigen::Isometry3d flat = Eigen::Isometry3d::Identity();
    flat.translation() = T.translation();
    flat.linear() =
      Eigen::AngleAxisd(yaw_of(T), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return flat;
  }

  static double yaw_of(const Eigen::Isometry3d & T)
  {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
  }

  void log_place(const char * label)
  {
    const Eigen::Vector3d p = place_estimate_.translation();
    RCLCPP_INFO(
      get_logger(), "%s: x=%.3f y=%.3f z=%.3f, yaw %.1f deg (%d samples)",
      label, p.x(), p.y(), p.z(), yaw_of(place_estimate_) * 180.0 / M_PI, place_samples_.count);
  }

  void report_placement_error()
  {
    const Eigen::Isometry3d cube = verify_samples_.mean();
    const Eigen::Vector2d error =
      cube.translation().head<2>() - place_estimate_.translation().head<2>();
    // The cube looks the same every 90 degrees, so the error is in [-45, 45] deg
    const double yaw_error = std::remainder(yaw_of(cube) - yaw_of(place_estimate_), M_PI / 2.0);

    RCLCPP_INFO(
      get_logger(),
      "Placement error: %.1f mm (x %.1f mm, y %.1f mm), rotation %.1f deg",
      error.norm() * 1000.0, error.x() * 1000.0, error.y() * 1000.0,
      yaw_error * 180.0 / M_PI);
  }

  // ======================= Motion helpers =======================

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
  double standoff_;
  double place_approach_height_;
  double place_push_;
  double retreat_height_;
  double gain_;
  double fast_speed_;
  double slow_speed_;
  double max_angular_;
  double open_width_;
  double observe_time_;
  int min_samples_;

  // State
  State state_ = State::WAIT_FOR_CUBE;
  rclcpp::Time state_start_;
  rclcpp::Time last_motion_time_;
  double last_hand_z_ = 1e9;
  bool has_cube_ = false;
  bool servo_holding_ = false;
  double finger_position_ = 0.0;
  std::string camera_frame_ = "camera_optical_frame";

  bool collecting_place_ = false;
  bool collecting_verify_ = false;
  PoseAverager place_samples_;
  PoseAverager verify_samples_;

  Eigen::Isometry3d cube_top_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d place_estimate_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d grasp_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pregrasp_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lift_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d scan_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d observe_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d place_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d preplace_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d retreat_pose_ = Eigen::Isometry3d::Identity();

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr holding_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_marker_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_marker_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickAndPlace>());
  rclcpp::shutdown();
  return 0;
}