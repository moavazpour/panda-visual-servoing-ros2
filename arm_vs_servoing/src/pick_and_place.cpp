#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

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
    // Geometry
    tcp_offset_ = declare_parameter("tcp_offset", 0.1034);       // hand origin -> fingertip centre [m]
    cube_size_ = declare_parameter("cube_size", 0.05);           // [m]
    grasp_height_offset_ = declare_parameter("grasp_height_offset", 0.015);  // [m]
    pregrasp_height_ = declare_parameter("pregrasp_height", 0.10);  // [m]
    lift_height_ = declare_parameter("lift_height", 0.15);       // [m]
    standoff_ = declare_parameter("standoff", 0.20);             // camera above a marker [m]
    place_approach_height_ = declare_parameter("place_approach_height", 0.10);  // [m]
    place_push_ = declare_parameter("place_push", 0.01);         // aim below contact [m]
    retreat_height_ = declare_parameter("retreat_height", 0.15);  // [m]

    // Motion
    gain_ = declare_parameter("gain", 1.5);
    fast_speed_ = declare_parameter("fast_speed", 0.08);         // [m/s]
    slow_speed_ = declare_parameter("slow_speed", 0.03);         // [m/s]
    max_angular_ = declare_parameter("max_angular_speed", 0.4);  // [rad/s]

    // Gripper
    open_width_ = declare_parameter("open_width", 0.04);         // [m]
    squeeze_time_ = declare_parameter("squeeze_time", 1.0);      // [s]

    // Probing grasp: close slowly, feel where the cube is, re-centre the hand, then grip
    probe_speed_ = declare_parameter("probe_speed", 0.02);       // finger speed [m/s]
    contact_force_ = declare_parameter("contact_force", 0.2);    // finger force = touching [N]
    contact_lag_ = declare_parameter("contact_lag", 0.002);      // finger blocked behind its target [m]
    centre_tolerance_ = declare_parameter("centre_tolerance", 0.001);  // [m]
    max_recentring_ = declare_parameter("max_recentring", 2);
    grip_squeeze_ = declare_parameter("grip_squeeze", 0.001);    // hold target inside contact [m]

    // Perception
    observe_time_ = declare_parameter("observe_time", 1.0);      // [s]
    min_samples_ = declare_parameter("min_samples", 10);

    // Experiment loop
    loop_ = declare_parameter("loop", true);
    randomize_ = declare_parameter("randomize", true);
    min_separation_ = declare_parameter("min_separation", 0.20);  // cube <-> shelf centres [m]
    // Arm posture: the 7th joint lets the elbow swing without moving the hand.
    // Every cycle gets a random elbow swivel inside this range [rad].
    swivel_range_ = declare_parameter("swivel_range", 0.7);
    success_tolerance_ = declare_parameter("success_tolerance", 0.02);  // [m]
    const std::string world_name = declare_parameter("world_name", std::string("vs_world"));
    const int seed = declare_parameter("seed", -1);
    rng_.seed(seed < 0 ? std::random_device{}() : static_cast<unsigned>(seed));

    // Overview pose: the robot's start position, both markers are visible from here
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
          const bool has_effort = i < msg.effort.size();
          if (msg.name[i] == "panda_finger_joint1") {
            finger1_ = msg.position[i];
            effort1_ = has_effort ? msg.effort[i] : NAN;
          } else if (msg.name[i] == "panda_finger_joint2") {
            finger2_ = msg.position[i];
            effort2_ = has_effort ? msg.effort[i] : NAN;
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
    swivel_pub_ = create_publisher<std_msgs::msg::Float64>("/elbow_swivel", 10);
    result_pub_ = create_publisher<std_msgs::msg::String>("/cycle_result", 10);
    set_pose_client_ = create_client<SetEntityPose>("/world/" + world_name + "/set_pose");

    timer_ = create_wall_timer(20ms, [this]() {step();});
    state_start_ = now();
    last_motion_time_ = now();
    enter(State::START);

    RCLCPP_INFO(get_logger(), "Pick and place started (loop=%s, randomize=%s)",
      loop_ ? "on" : "off", randomize_ ? "on" : "off");
  }

private:
  enum class State
  {
    START,
    RANDOMIZE,
    SETTLE,
    WAIT_FOR_CUBE,
    OPEN_GRIPPER,
    PRE_GRASP,
    DESCEND,
    PROBE,
    RECENTRE,
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
    GO_TO_VERIFY,
    VERIFY,
    IDLE
  };

  // Random layout ranges [m]: both markers must be visible from the scan pose
  static constexpr double kCubeXMin = 0.40, kCubeXMax = 0.47, kCubeYMax = 0.15;
  static constexpr double kShelfXMin = 0.40, kShelfXMax = 0.48, kShelfYMax = 0.13;
  static constexpr double kCubeZ = 0.176;   // table top 0.15 + half cube + 1 mm
  static constexpr double kShelfZ = 0.20;   // table top 0.15 + half shelf height

  // ======================= State machine =======================

  void step()
  {
    // Nothing can be done before the robot pose is known
    if (!tf_buffer_.canTransform("panda_link0", "panda_hand", tf2::TimePointZero)) {
      return;
    }
    publish_servo_enable();
    publish_swivel();

    // While the cube is in the hand, keep it centred between the fingers
    if (holding_cube_) {
      hold_centered();
    }

    switch (state_) {
      // ---------- New cycle ----------
      case State::START:
        if (go_to(scan_pose_, fast_speed_)) {
          set_state(randomize_ ? State::RANDOMIZE : State::SETTLE);
        }
        break;

      case State::RANDOMIZE:
        stop();
        if (!layout_sent_) {
          // A freshly started node needs a moment to discover the service
          if (set_pose_client_->service_is_ready()) {
            randomize_scene();
            layout_sent_ = true;
          } else if (seconds_in_state() > 10.0) {
            RCLCPP_ERROR(
              get_logger(), "Service %s not available. Is the bridge running?",
              set_pose_client_->get_service_name());
            set_state(State::IDLE);
          }
        } else if (request_failed_) {
          RCLCPP_ERROR(get_logger(), "Could not move the objects, stopping the experiment");
          set_state(State::IDLE);
        } else if (pending_requests_ == 0) {
          set_state(State::SETTLE);
        } else if (seconds_in_state() > 15.0) {
          RCLCPP_ERROR(get_logger(), "No answer from Gazebo, stopping the experiment");
          set_state(State::IDLE);
        }
        break;

      case State::SETTLE:
        // The hand is still empty: a good moment to find the shelf (coarse estimate)
        stop();
        if (seconds_in_state() > 2.0) {
          if (place_samples_.count >= min_samples_) {
            place_estimate_ = place_samples_.mean();
            has_place_ = true;
          } else {
            RCLCPP_WARN(get_logger(), "Shelf not seen before the pick, will look for it later");
          }
          set_state(State::WAIT_FOR_CUBE);
        }
        break;

      // ---------- Pick ----------
      case State::WAIT_FOR_CUBE:
        // The servo is in control now; this node only watches
        if (servo_holding_ && has_cube_) {
          if (seconds_in_state() > 1.0) {
            plan_grasp();
            set_state(State::OPEN_GRIPPER);
          }
        } else {
          state_start_ = now();
        }
        if ((now() - wait_started_).seconds() > 40.0) {
          RCLCPP_ERROR(get_logger(), "The servo did not reach the cube in time");
          finish_cycle(false, 0.0);
        }
        break;

      case State::OPEN_GRIPPER:
        stop();
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
          set_state(State::PROBE);
        }
        break;

      case State::PROBE:
        // Close both fingers slowly. A finger that touches the cube stops where it is,
        // so the cube is not pushed. When both touch, we know where the cube is in the hand.
        stop();
        probe_finger(finger1_, effort1_, probe_target1_, touched1_, touch1_);
        probe_finger(finger2_, effort2_, probe_target2_, touched2_, touch2_);
        command_fingers(probe_target1_, probe_target2_);

        if (touched1_ && touched2_) {
          // Finger 1 moves along the hand's +y axis, finger 2 along -y.
          // The cube spans from -touch2 to +touch1, so its centre is at (touch1 - touch2) / 2.
          const double offset = (touch1_ - touch2_) / 2.0;
          if (recentrings_ == 0) {
            grasp_offset_ = offset;
          }
          RCLCPP_INFO(
            get_logger(), "Fingers touch the cube: width %.1f mm, cube %.1f mm off centre",
            (touch1_ + touch2_) * 1000.0, offset * 1000.0);

          if (std::abs(offset) > centre_tolerance_ && recentrings_ < max_recentring_) {
            // Move the hand sideways by the offset, then feel again
            const Eigen::Vector3d shift = current_hand_pose().linear().col(1) * offset;
            recentre_pose_ = current_hand_pose();
            recentre_pose_.translation() += shift;
            lift_pose_.translation() += shift;
            ++recentrings_;
            set_state(State::RECENTRE);
          } else {
            set_state(State::CLOSE_GRIPPER);
          }
        } else if (finger1_ + finger2_ < 0.01) {
          RCLCPP_ERROR(get_logger(), "Fingers closed on nothing, the cube was missed");
          finish_cycle(false, 0.0);
        } else if (seconds_in_state() > 6.0) {
          RCLCPP_WARN(get_logger(), "Probing took too long, gripping anyway");
          set_state(State::CLOSE_GRIPPER);
        }
        break;

      case State::RECENTRE:
        // Open first, so no finger drags the cube, then move the hand
        command_gripper(open_width_);
        if (seconds_in_state() < 0.5) {
          stop();
        } else if (go_to(recentre_pose_, slow_speed_, 0.0003) || seconds_in_state() > 4.0) {
          // The correction is only a few mm, so it needs a finer tolerance than other moves
          set_state(State::PROBE);
        }
        break;

      case State::CLOSE_GRIPPER:
        stop();
        if (seconds_in_state() < squeeze_time_) {
          // Phase 1: both fingers close together
          command_gripper(0.0);
        } else {
          // Phase 2: hold_centered() takes over and pulls the cube to the centre
          holding_cube_ = true;
          if (seconds_in_state() > squeeze_time_ + 1.0) {
            RCLCPP_INFO(
              get_logger(),
              "Fingers %.1f mm / %.1f mm, opening %.1f mm, cube %.1f mm off centre",
              finger1_ * 1000.0, finger2_ * 1000.0, (finger1_ + finger2_) * 1000.0,
              (finger1_ - finger2_) / 2.0 * 1000.0);
            if (finger1_ + finger2_ < 0.01) {
              RCLCPP_ERROR(get_logger(), "Fingers closed on nothing, the cube was missed");
              finish_cycle(false, 0.0);
            } else {
              set_state(State::LIFT);
            }
          }
        }
        break;

      case State::LIFT:
        if (go_to(lift_pose_, slow_speed_)) {
          if (has_place_) {
            // Shelf already known: go straight above it
            observe_pose_ = camera_above(place_estimate_, standoff_);
            set_state(State::GO_ABOVE_PLACE);
          } else {
            set_state(State::GO_TO_SCAN);
          }
        }
        break;

      // ---------- Find the place location (fallback if it was not seen before) ----------
      case State::GO_TO_SCAN:
        if (go_to(scan_pose_, fast_speed_)) {
          set_state(State::LOOK_FOR_PLACE);
        }
        break;

      case State::LOOK_FOR_PLACE:
        stop();
        if (seconds_in_state() > observe_time_ && place_samples_.count >= min_samples_) {
          place_estimate_ = place_samples_.mean();
          observe_pose_ = camera_above(place_estimate_, standoff_);
          set_state(State::GO_ABOVE_PLACE);
        } else if (seconds_in_state() > 5.0) {
          RCLCPP_ERROR(get_logger(), "Place marker not visible from the scan pose");
          finish_cycle(false, 0.0);
        }
        break;

      // ---------- Measure the place location from close by ----------
      case State::GO_ABOVE_PLACE:
        if (go_to(observe_pose_, fast_speed_)) {
          set_state(State::MEASURE_PLACE);
        }
        break;

      case State::MEASURE_PLACE:
        stop();
        if (seconds_in_state() > observe_time_ && place_samples_.count >= min_samples_) {
          place_estimate_ = place_samples_.mean();
          plan_place();
          set_state(State::PRE_PLACE);
        } else if (seconds_in_state() > 5.0) {
          RCLCPP_ERROR(get_logger(), "Place marker lost when looking from close by");
          finish_cycle(false, 0.0);
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
          // Look at the cube from the same distance the shelf marker was measured from:
          // the camera's depth error is then the same in both measurements and cancels.
          verify_pose_ = camera_above(place_estimate_, standoff_ + cube_size_);
          set_state(State::GO_TO_VERIFY);
        }
        break;

      case State::GO_TO_VERIFY:
        if (go_to(verify_pose_, slow_speed_)) {
          set_state(State::VERIFY);
        }
        break;

      // ---------- Measure how well we did ----------
      case State::VERIFY:
        stop();
        if (seconds_in_state() > observe_time_ && verify_samples_.count >= min_samples_) {
          const double error = report_placement_error();
          finish_cycle(error < success_tolerance_, error);
        } else if (seconds_in_state() > 4.0) {
          RCLCPP_ERROR(get_logger(), "Cube not visible on the shelf after placing");
          finish_cycle(false, 0.0);
        }
        break;

      case State::IDLE:
        stop();
        break;
    }
  }

  void set_state(State next)
  {
    static const char * names[] = {
      "START", "RANDOMIZE", "SETTLE", "WAIT_FOR_CUBE", "OPEN_GRIPPER", "PRE_GRASP",
      "DESCEND", "PROBE", "RECENTRE", "CLOSE_GRIPPER", "LIFT", "GO_TO_SCAN", "LOOK_FOR_PLACE", "GO_ABOVE_PLACE",
      "MEASURE_PLACE", "PRE_PLACE", "PLACE_DESCEND", "RELEASE", "RETREAT", "GO_TO_VERIFY",
      "VERIFY", "IDLE"};
    RCLCPP_INFO(
      get_logger(), "%s -> %s",
      names[static_cast<int>(state_)], names[static_cast<int>(next)]);
    state_ = next;
    state_start_ = now();
    last_motion_time_ = now();
    last_hand_z_ = 1e9;
    collecting_place_ = false;
    collecting_verify_ = false;
    enter(next);
  }

  // Actions that run once when a state is entered
  void enter(State s)
  {
    switch (s) {
      case State::START:
        servo_enabled_ = false;
        holding_cube_ = false;
        has_place_ = false;
        cycle_start_ = now();
        choose_swivel();
        command_gripper(open_width_);
        break;
      case State::RANDOMIZE:
        layout_sent_ = false;
        pending_requests_ = 0;
        request_failed_ = false;
        break;
      case State::SETTLE:
      case State::LOOK_FOR_PLACE:
      case State::MEASURE_PLACE:
        place_samples_.reset();
        collecting_place_ = true;
        break;
      case State::WAIT_FOR_CUBE:
        has_cube_ = false;
        servo_holding_ = false;
        servo_enabled_ = true;
        wait_started_ = now();
        break;
      case State::OPEN_GRIPPER:
        servo_enabled_ = false;  // this node takes over from here
        recentrings_ = 0;
        grasp_offset_ = 0.0;
        break;
      case State::PROBE:
        touched1_ = false;
        touched2_ = false;
        probe_target1_ = finger1_;
        probe_target2_ = finger2_;
        break;
      case State::CLOSE_GRIPPER:
        holding_cube_ = false;
        break;
      case State::RELEASE:
        holding_cube_ = false;   // stop centring, let the fingers open
        break;
      case State::VERIFY:
        verify_samples_.reset();
        collecting_verify_ = true;
        break;
      case State::IDLE:
        holding_cube_ = false;
        break;
      default:
        break;
    }
  }

  double seconds_in_state() {return (now() - state_start_).seconds();}

  void finish_cycle(bool success, double error)
  {
    ++cycles_;
    if (success) {
      ++successes_;
      error_sum_ += error;
      max_error_ = std::max(max_error_, error);
    }
    const double mean_error = successes_ > 0 ? error_sum_ / successes_ : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "===== Cycle %d: %s | success %d/%d (%.0f%%) | mean error %.1f mm | max error %.1f mm =====",
      cycles_, success ? "SUCCESS" : "FAILED", successes_, cycles_,
      100.0 * successes_ / cycles_, mean_error * 1000.0, max_error_ * 1000.0);

    // Report the cycle, so an external tool can benchmark it
    char json[256];
    std::snprintf(
      json, sizeof(json),
      "{\"cycle\": %d, \"success\": %s, \"estimated_error_mm\": %.2f, \"duration_s\": %.1f, "
      "\"swivel_deg\": %.1f, \"grasp_offset_mm\": %.2f}",
      cycles_, success ? "true" : "false", error * 1000.0, (now() - cycle_start_).seconds(),
      swivel_ * 180.0 / M_PI, grasp_offset_ * 1000.0);
    std_msgs::msg::String result;
    result.data = json;
    result_pub_->publish(result);

    set_state(loop_ ? State::START : State::IDLE);
  }

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
      RCLCPP_INFO(get_logger(), "Contact %.1f mm above the target", remaining * 1000.0);
      return true;
    }
    return false;
  }

  // ======================= Scene randomization =======================

  void randomize_scene()
  {
    std::uniform_real_distribution<double> cube_x(kCubeXMin, kCubeXMax);
    std::uniform_real_distribution<double> cube_y(-kCubeYMax, kCubeYMax);
    std::uniform_real_distribution<double> shelf_x(kShelfXMin, kShelfXMax);
    std::uniform_real_distribution<double> shelf_y(-kShelfYMax, kShelfYMax);
    std::uniform_real_distribution<double> yaw(-M_PI / 4.0, M_PI / 4.0);

    // Draw random layouts until cube and shelf are far enough apart
    double cx = 0.0, cy = 0.0, sx = 0.0, sy = 0.0;
    bool found = false;
    for (int i = 0; i < 500 && !found; ++i) {
      cx = cube_x(rng_);
      cy = cube_y(rng_);
      sx = shelf_x(rng_);
      sy = shelf_y(rng_);
      found = std::hypot(cx - sx, cy - sy) >= min_separation_;
    }
    if (!found) {
      RCLCPP_ERROR(get_logger(), "Could not find a valid random layout");
      request_failed_ = true;
      return;
    }

    const double cube_yaw = yaw(rng_);
    const double shelf_yaw = yaw(rng_);
    send_pose("aruco_cube", cx, cy, kCubeZ, cube_yaw);
    send_pose("place_shelf", sx, sy, kShelfZ, shelf_yaw);

    RCLCPP_INFO(
      get_logger(), "New layout: cube x=%.3f y=%.3f yaw %.0f deg | shelf x=%.3f y=%.3f yaw %.0f deg",
      cx, cy, cube_yaw * 180.0 / M_PI, sx, sy, shelf_yaw * 180.0 / M_PI);
  }

  void send_pose(const std::string & name, double x, double y, double z, double yaw)
  {
    auto request = std::make_shared<SetEntityPose::Request>();
    request->entity.name = name;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose.position.x = x;
    request->pose.position.y = y;
    request->pose.position.z = z;
    request->pose.orientation.z = std::sin(yaw / 2.0);
    request->pose.orientation.w = std::cos(yaw / 2.0);

    ++pending_requests_;
    set_pose_client_->async_send_request(
      request,
      [this, name](rclcpp::Client<SetEntityPose>::SharedFuture future) {
        --pending_requests_;
        if (!future.get()->success) {
          RCLCPP_ERROR(get_logger(), "Gazebo could not move %s", name.c_str());
          request_failed_ = true;
        }
      });
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
  }

  void plan_place()
  {
    const Eigen::Vector3d p = place_estimate_.translation();
    const double contact_z = p.z() + cube_size_ / 2.0 + grasp_height_offset_ + tcp_offset_;

    place_pose_ = Eigen::Isometry3d::Identity();
    place_pose_.linear() = aligned_hand_rotation(yaw_of(place_estimate_));
    place_pose_.translation() = Eigen::Vector3d(p.x(), p.y(), contact_z - place_push_);

    preplace_pose_ = place_pose_;
    preplace_pose_.translation().z() = contact_z + place_approach_height_;

    retreat_pose_ = place_pose_;
    retreat_pose_.translation().z() = contact_z + retreat_height_;
  }

  // Hand pointing down, fingers parallel to a face of a square with this yaw.
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

  // Hand pose that puts the camera 'height' above a marker, looking straight down
  Eigen::Isometry3d camera_above(const Eigen::Isometry3d & marker, double height)
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
      goal.translate(Eigen::Vector3d(0.0, 0.0, height));
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

  // Logs the placement error and returns its size [m]
  double report_placement_error()
  {
    const Eigen::Isometry3d cube = verify_samples_.mean();
    const Eigen::Vector2d error =
      cube.translation().head<2>() - place_estimate_.translation().head<2>();
    const double yaw_error = std::remainder(yaw_of(cube) - yaw_of(place_estimate_), M_PI / 2.0);

    RCLCPP_INFO(
      get_logger(), "Placement error: %.1f mm (x %.1f mm, y %.1f mm), rotation %.1f deg",
      error.norm() * 1000.0, error.x() * 1000.0, error.y() * 1000.0,
      yaw_error * 180.0 / M_PI);
    return error.norm();
  }

  // ======================= Motion helpers =======================

  bool go_to(const Eigen::Isometry3d & target, double max_speed, double tolerance = 0.002)
  {
    const Eigen::Isometry3d hand = current_hand_pose();

    const Eigen::Vector3d position_error = target.translation() - hand.translation();
    const Eigen::AngleAxisd rotation_diff(target.linear() * hand.linear().transpose());
    const Eigen::Vector3d rotation_error = rotation_diff.angle() * rotation_diff.axis();

    if (position_error.norm() < tolerance && rotation_diff.angle() < 0.017) {
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

  // Software version of the gear that couples the real Franka fingers.
  // Both fingers get the same target, slightly inside their average position.
  // The finger the cube has moved towards is further from that target and pushes
  // harder, the other one backs off, so the cube is pushed back to the centre.
  void hold_centered()
  {
    const double target = (finger1_ + finger2_) / 2.0 - grip_squeeze_;
    command_gripper(std::max(target, 0.0));
  }

  // One finger during probing: move its target inwards until it touches the cube,
  // then keep the target where the finger stopped.
  void probe_finger(
    double position, double effort, double & target, bool & touched, double & touch_position)
  {
    if (touched) {
      return;
    }
    // Touch = the finger feels a force, or it is stuck behind its moving target.
    // The force is ignored for a moment after the start, while the finger accelerates.
    const bool force_contact =
      seconds_in_state() > 0.3 && std::isfinite(effort) && std::abs(effort) > contact_force_;
    const bool blocked = position - target > contact_lag_;
    if (force_contact || blocked) {
      touched = true;
      touch_position = position;
      target = position;
      RCLCPP_INFO(
        get_logger(), "Finger touched at %.1f mm (force %.2f N%s)", position * 1000.0, effort,
        blocked && !force_contact ? ", detected by position" : "");
      return;
    }
    target = std::max(0.0, target - probe_speed_ * 0.02);  // 20 ms timer period
  }

  // Same target for both fingers
  void command_gripper(double finger_position)
  {
    command_fingers(finger_position, finger_position);
  }

  // Separate target for each finger
  void command_fingers(double finger1, double finger2)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {finger1, finger2};
    gripper_pub_->publish(msg);
  }

  // A new arm posture for this cycle. The hand path does not change, only the elbow.
  void choose_swivel()
  {
    if (randomize_) {
      std::uniform_real_distribution<double> swivel(-swivel_range_, swivel_range_);
      swivel_ = swivel(rng_);
    } else {
      swivel_ = 0.0;
    }
    RCLCPP_INFO(get_logger(), "Elbow swivel for this cycle: %.0f deg", swivel_ * 180.0 / M_PI);
  }

  // Sent every tick, so the controller gets it even if it starts later than this node
  void publish_swivel()
  {
    std_msgs::msg::Float64 msg;
    msg.data = swivel_;
    swivel_pub_->publish(msg);
  }

  // Sent every cycle, so the servo gets it even if it starts later than this node
  void publish_servo_enable()
  {
    std_msgs::msg::Bool msg;
    msg.data = servo_enabled_;
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
  double squeeze_time_;
  double grip_squeeze_;
  double probe_speed_;
  double contact_force_;
  double contact_lag_;
  double centre_tolerance_;
  int max_recentring_;
  double observe_time_;
  int min_samples_;
  bool loop_;
  bool randomize_;
  double swivel_range_;
  double swivel_ = 0.0;
  double min_separation_;
  double success_tolerance_;

  // State machine
  State state_ = State::START;
  rclcpp::Time state_start_;
  rclcpp::Time wait_started_;
  rclcpp::Time cycle_start_;
  rclcpp::Time last_motion_time_;
  double last_hand_z_ = 1e9;
  bool servo_enabled_ = false;
  bool holding_cube_ = false;
  bool has_place_ = false;

  // Probing grasp
  double effort1_ = NAN;
  double effort2_ = NAN;
  double probe_target1_ = 0.0;
  double probe_target2_ = 0.0;
  bool touched1_ = false;
  bool touched2_ = false;
  double touch1_ = 0.0;
  double touch2_ = 0.0;
  int recentrings_ = 0;
  double grasp_offset_ = 0.0;
  Eigen::Isometry3d recentre_pose_ = Eigen::Isometry3d::Identity();

  // Inputs
  bool has_cube_ = false;
  bool servo_holding_ = false;
  double finger1_ = 0.0;
  double finger2_ = 0.0;
  std::string camera_frame_ = "camera_optical_frame";
  bool collecting_place_ = false;
  bool collecting_verify_ = false;
  PoseAverager place_samples_;
  PoseAverager verify_samples_;

  // Poses
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
  Eigen::Isometry3d verify_pose_ = Eigen::Isometry3d::Identity();

  // Randomization and statistics
  std::mt19937 rng_;
  bool layout_sent_ = false;
  int pending_requests_ = 0;
  bool request_failed_ = false;
  int cycles_ = 0;
  int successes_ = 0;
  double error_sum_ = 0.0;
  double max_error_ = 0.0;

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
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swivel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Client<SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickAndPlace>());
  rclcpp::shutdown();
  return 0;
}