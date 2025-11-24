#include "ttbot_controller/stanley_controller.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


//pkg name
namespace
{
  constexpr const char* kPackageName = "ttbot_controller";
}

StanleyController::StanleyController()
: Node("stanley_controller"),
  last_index_(0)    // khởi tạo last_index_
{

  // Declare parameters
  this->declare_parameter("K", 1.0); // Stanley gain
  this->declare_parameter("L", 0.8); // wheel base
  this->declare_parameter("desired_speed", 0.8);
  this->declare_parameter("max_steer_deg", 60.0);
  this->declare_parameter("path_file", "path.csv");

  // Load parameters
  K_             = this->get_parameter("K").as_double();
  L_             = this->get_parameter("L").as_double();
  desired_speed_ = this->get_parameter("desired_speed").as_double();
  double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
  path_file_     = this->get_parameter("path_file").as_string();

  max_steer_ = max_steer_deg * M_PI / 180.0;

  // Load path file
  loadPathFromCSV();

  // Subscribers & Publishers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ackermann_controller/odom", 10,
      std::bind(&StanleyController::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_controller/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(),
              "Stanley Controller initialized. K=%.3f, L=%.3f, v=%.3f, max_steer=%.1f deg, path_points=%zu",
              K_, L_, desired_speed_, max_steer_deg, path_points_.size());
}

// ================================
// Load CSV Path File
// ================================
void StanleyController::loadPathFromCSV()
{
  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory(kPackageName);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to get share directory for package '%s': %s",
                 kPackageName, e.what());
    return;
  }
    // CSV trong share/ttbot_controller/path/
    std::string csv_path = share_dir + "/path/" + path_file_;

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open path file: %s", csv_path.c_str());
    return;
  }

  path_points_.clear();
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    double px, py;
    char comma;
    if (ss >> px >> comma >> py) {
      path_points_.emplace_back(px, py);
    }
  }
  file.close();

  RCLCPP_INFO(this->get_logger(),
              "Loaded %zu path points from %s",
              path_points_.size(), csv_path.c_str());
}

// ================================
// Normalize angle to [-pi, pi]
// ================================
double StanleyController::normalizeAngle(double angle)
{
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// ================================
// Callback: Process /odom
// ================================
void StanleyController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (path_points_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No path loaded yet!");
    return;
  }

  // ==== trạng thái hiện tại ====
  double x   = msg->pose.pose.position.x;
  double y   = msg->pose.pose.position.y;
  double yaw = tf2::getYaw(msg->pose.pose.orientation);

  // ==== 1) Tìm điểm gần nhất, CHỈ TÌM Ở PHÍA TRƯỚC ====
  size_t n = path_points_.size();

  if (last_index_ >= n) {
    last_index_ = n - 1;
  }

  const size_t SEARCH_WINDOW = 40;                    // có thể tăng/giảm
  size_t start_idx = last_index_;
  size_t end_idx   = std::min(last_index_ + SEARCH_WINDOW, n - 1);

  size_t closest_index = start_idx;
  double min_dist_sq   = std::numeric_limits<double>::infinity();

  for (size_t i = start_idx; i <= end_idx; ++i) {
    double dx = x - path_points_[i].first;
    double dy = y - path_points_[i].second;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq   = dist_sq;
      closest_index = i;
    }
  }

  // cập nhật lại, LẦN SAU CHỈ TÌM TỪ ĐÂY TRỞ VỀ SAU
  last_index_ = closest_index;

  // ==== 2) Tính yaw của path tại điểm đó ====
  double path_yaw;
  if (closest_index < n - 1) {
    double next_x = path_points_[closest_index + 1].first;
    double next_y = path_points_[closest_index + 1].second;
    path_yaw = std::atan2(
        next_y - path_points_[closest_index].second,
        next_x - path_points_[closest_index].first);
  } else {
    // điểm cuối: dùng đoạn nối với điểm trước nó
    double prev_x = path_points_[closest_index - 1].first;
    double prev_y = path_points_[closest_index - 1].second;
    path_yaw = std::atan2(
        path_points_[closest_index].second - prev_y,
        path_points_[closest_index].first  - prev_x);
  }

  // ==== 3) Heading error ====
  double heading_error = normalizeAngle(path_yaw - yaw);

  // ==== 4) Cross-track error (x sang trước, y sang trái) ====
  double dx = x - path_points_[closest_index].first;
  double dy = y - path_points_[closest_index].second;
  double cross_error = (-std::sin(path_yaw) * dx) + (std::cos(path_yaw) * dy);

  // ==== 5) Giảm tốc độ gần cuối path (cho đỡ lao quá) ====
  double v_cmd = desired_speed_;
  if (closest_index > n - 5) {
    v_cmd = std::min(desired_speed_, 0.5);    // chậm lại khi gần cuối
  }

  // ==== 6) Stanley steering ====
  double steer_angle = heading_error + std::atan2(K_ * cross_error, v_cmd);
  if (steer_angle > max_steer_)  steer_angle = max_steer_;
  if (steer_angle < -max_steer_) steer_angle = -max_steer_;

  double yaw_rate = v_cmd * std::tan(steer_angle) / L_;

  // ==== 7) Publish cmd_vel ====
  geometry_msgs::msg::TwistStamped cmd_msg;
  cmd_msg.header.stamp = this->now();
  cmd_msg.header.frame_id = "base_link";
  cmd_msg.twist.linear.x  = v_cmd;
  cmd_msg.twist.angular.z = yaw_rate;
  cmd_pub_->publish(cmd_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "cte=%.2f, heading_error=%.2f deg, steer=%.2f deg, yaw_rate=%.2f rad/s, idx=%zu/%zu",
      cross_error,
      heading_error * 180.0 / M_PI,
      steer_angle * 180.0 / M_PI,
      yaw_rate,
      closest_index, n-1);
}


// ================================
// Main
// ================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StanleyController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
