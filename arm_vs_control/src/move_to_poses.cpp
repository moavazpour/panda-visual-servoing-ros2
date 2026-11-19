#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class MoveToPoses : public rclcpp::Node
{
public:
  MoveToPoses()
  : Node("move_to_poses")
  {
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/arm_controller/follow_joint_trajectory");
  }

  // Send one goal and block until the robot has finished moving.
  bool move_to(const std::vector<double> & positions, double duration_s)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available. Is the simulation running?");
      return false;
    }

    // 1. Build the goal: joint names + one target point
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    goal.trajectory.points.push_back(point);

    // 2. Send the goal and wait until the controller accepts or rejects it
    auto goal_future = client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the controller");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Goal accepted, moving...");

    // 3. Wait until the motion is finished
    auto result_future = client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Motion failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Motion finished");
    return true;
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;

  const std::vector<std::string> joint_names_ = {
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoses>();

  // Each pose = 7 joint angles in radians
  const std::vector<double> home = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  const std::vector<double> left = {0.8, -0.4, 0.0, -2.0, 0.0, 1.6, 0.785};
  const std::vector<double> right = {-0.8, -0.4, 0.0, -2.0, 0.0, 1.6, 0.785};
  const std::vector<double> look_down = {0.0, 0.2, 0.0, -2.0, 0.0, 2.2, 0.785};

  const std::vector<std::vector<double>> poses = {home, left, right, look_down, home};

  for (const auto & pose : poses) {
    if (!node->move_to(pose, 3.0)) {
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}