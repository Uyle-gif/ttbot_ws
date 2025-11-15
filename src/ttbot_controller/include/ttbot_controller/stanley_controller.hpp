#ifndef TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"   // THAY CHO twist.hpp
#include "tf2/utils.h"

class StanleyController : public rclcpp::Node
{
public:
  StanleyController();

private:
  // ==== Callbacks ====
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ==== Helpers ====
  void loadPathFromCSV();
  static double normalizeAngle(double angle);
  


  
  // ==== Parameters ====
  double K_;              // Stanley gain
  double L_;              // wheel base (bicycle model)
  double desired_speed_;  // m/s
  double max_steer_;      // max steering (rad)
  std::string path_file_; // relative name of csv file in share/

  // ==== ROS ====
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

  // ==== Path data ====
  // path_points_[i] = (x_i, y_i)
  std::vector<std::pair<double, double>> path_points_;
};

#endif  // TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_
