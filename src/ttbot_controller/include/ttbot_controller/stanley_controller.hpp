#ifndef TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>

class StanleyController : public rclcpp::Node
{
public:
    StanleyController();
    ~StanleyController();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    size_t findClosestPoint(double front_x, double front_y);
    double computeSteering(double front_x, double front_y, double yaw, double v);

    // --- Parameters ---
    double desired_speed_; 
    double min_speed_;         // <--- MỚI: Tốc độ tối thiểu khi cua gắt
    double slow_down_factor_;  // <--- MỚI: Hệ số giảm tốc (càng lớn càng giảm mạnh)
    
    double wheel_base_;    
    double max_steer_;     
    double k_gain_;        
    double k_soft_;        
    double goal_tolerance_;
    
    // --- State ---
    bool reached_goal_;
    bool has_path_ = false;
    size_t current_index_;
    std::vector<std::pair<double, double>> path_points_;

    // --- Communication ---
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_cte_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_heading_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
};

#endif