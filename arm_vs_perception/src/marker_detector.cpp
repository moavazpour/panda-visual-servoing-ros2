#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class MarkerDetector : public rclcpp::Node
{
public:
  MarkerDetector()
  : Node("marker_detector")
  {
    marker_id_ = declare_parameter("marker_id", 0);
    marker_length_ = declare_parameter("marker_length", 0.04);  // black square side [m]

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    // Keep nested candidates even when the white frame around the marker is thin
    detector_params_->minMarkerDistanceRate = 0.01;

    // Also find small markers, when the camera is far away
    detector_params_->minMarkerPerimeterRate = 0.02;

    // Larger thresholding windows, for big markers when the camera is close
    detector_params_->adaptiveThreshWinSizeMin = 3;
    detector_params_->adaptiveThreshWinSizeMax = 53;
    detector_params_->adaptiveThreshWinSizeStep = 10;
    
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/wrist_camera/camera_info", 10,
      [this](const sensor_msgs::msg::CameraInfo & msg) {on_camera_info(msg);});

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/wrist_camera/image", 10,
      [this](const sensor_msgs::msg::Image & msg) {on_image(msg);});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/marker_debug_image", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Looking for marker id %d (%.3f m)", marker_id_, marker_length_);
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo & msg)
  {
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = msg.k[i];
    }

    if (msg.d.empty()) {
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      dist_coeffs_ = cv::Mat(msg.d, true).reshape(1, 1);
    }

    if (!has_camera_info_) {
      RCLCPP_INFO(
        get_logger(), "Camera info received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
        msg.k[0], msg.k[4], msg.k[2], msg.k[5]);
    }
    has_camera_info_ = true;
  }

  void on_image(const sensor_msgs::msg::Image & msg)
  {
    if (!has_camera_info_) {
      return;
    }

    // 1. ROS image -> OpenCV image
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat & frame = cv_image->image;

    // 2. Find markers, and also keep the candidates that were rejected
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);

    // Rejected candidates in red, accepted markers in green
    if (!rejected.empty()) {
      cv::aruco::drawDetectedMarkers(frame, rejected, cv::noArray(), cv::Scalar(0, 0, 255));
    }
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }

    // 3. Estimate the pose of our marker
    bool found = false;
    double distance = 0.0;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != marker_id_) {
        continue;
      }

      cv::Vec3d rvec, tvec;
      if (!estimate_pose(corners[i], rvec, tvec)) {
        continue;
      }

      cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvec, tvec, marker_length_ * 0.75);
      publish_pose(msg.header, rvec, tvec);
      found = true;
      distance = cv::norm(tvec);
    }

    // 4. Status text on top of the image
    char status[128];
    cv::Scalar color;
    if (found) {
      std::snprintf(status, sizeof(status), "MARKER %d  distance %.3f m", marker_id_, distance);
      color = cv::Scalar(0, 200, 0);
    } else {
      std::snprintf(status, sizeof(status), "NO MARKER");
      color = cv::Scalar(0, 0, 255);
    }
    cv::putText(frame, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

    char counts[128];
    std::snprintf(
      counts, sizeof(counts), "detected: %zu  rejected: %zu", ids.size(), rejected.size());
    cv::putText(
      frame, counts, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    // 5. Publish the image with drawings
    debug_pub_->publish(*cv_image->toImageMsg());
  }

  bool estimate_pose(
    const std::vector<cv::Point2f> & image_corners, cv::Vec3d & rvec, cv::Vec3d & tvec)
  {
    // Corners of the marker in its own frame, same order as ArUco returns them:
    // top-left, top-right, bottom-right, bottom-left
    const float h = static_cast<float>(marker_length_ / 2.0);
    const std::vector<cv::Point3f> object_corners = {
      {-h, h, 0.0f}, {h, h, 0.0f}, {h, -h, 0.0f}, {-h, -h, 0.0f}};

    return cv::solvePnP(
      object_corners, image_corners, camera_matrix_, dist_coeffs_,
      rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
  }

  void publish_pose(
    const std_msgs::msg::Header & header, const cv::Vec3d & rvec, const cv::Vec3d & tvec)
  {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    tf2::Matrix3x3 rotation(
      R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    tf2::Quaternion q;
    rotation.getRotation(q);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = tvec[0];
    pose.pose.position.y = tvec[1];
    pose.pose.position.z = tvec[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = header;
    transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id_);
    transform.transform.translation.x = tvec[0];
    transform.transform.translation.y = tvec[1];
    transform.transform.translation.z = tvec[2];
    transform.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  // Settings
  int marker_id_;
  double marker_length_;

  // OpenCV state
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool has_camera_info_ = false;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerDetector>());
  rclcpp::shutdown();
  return 0;
}