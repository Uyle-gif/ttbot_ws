#include "ttbot_controller/stanley_controller.hpp"
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static double deg2rad(double deg) { return deg * M_PI / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

StanleyController::StanleyController()
: Node("stanley_controller")
{
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("wheel_base", 0.65);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("k_gain", 2.0);  
    this->declare_parameter("k_soft", 1.0);  

    desired_speed_ = this->get_parameter("desired_speed").as_double();
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
    max_steer_     = deg2rad(max_steer_deg);
    
    k_gain_ = this->get_parameter("k_gain").as_double();
    k_soft_ = this->get_parameter("k_soft").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    current_index_ = 0;
    has_path_ = false;
    reached_goal_ = false; 

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&StanleyController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mpc_path", 1, 
        std::bind(&StanleyController::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann_controller/cmd_vel", 10);

    error_cte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/stanley/error/cte", 10);
    error_heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/stanley/error/heading", 10);

        
    RCLCPP_INFO(this->get_logger(), 
        "Stanley RESTORED: MaxSteer=%.1f deg, Speed=%.2f m/s, K=%.2f", max_steer_deg, desired_speed_, k_gain_);
}

StanleyController::~StanleyController() {}

void StanleyController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) return;

    path_points_.clear();
    path_points_.reserve(msg->poses.size());
    for (const auto &pose_stamped : msg->poses) {
        path_points_.emplace_back(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y
        );
    }

    current_index_ = 0;
    has_path_ = true;
    reached_goal_ = false; 

    RCLCPP_INFO(this->get_logger(), "--> PATH RECEIVED: %zu points. Resetting...", path_points_.size());
}

size_t StanleyController::findClosestPoint(double front_x, double front_y)
{
    if (path_points_.empty()) return 0;

    size_t closest_idx = current_index_;
    double min_dist_sq = std::numeric_limits<double>::infinity();
    
    size_t search_limit = std::min(current_index_ + 100, path_points_.size());
    if (current_index_ == 0) search_limit = path_points_.size();

    for (size_t i = current_index_; i < search_limit; ++i) {
        double dx = front_x - path_points_[i].first;
        double dy = front_y - path_points_[i].second;
        double d_sq = dx*dx + dy*dy;

        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            closest_idx = i;
        }
    }
    
    current_index_ = closest_idx;
    return closest_idx;
}

double StanleyController::computeSteering(double front_x, double front_y, double yaw, double v)
{
    size_t idx = findClosestPoint(front_x, front_y);
    double map_x = path_points_[idx].first;
    double map_y = path_points_[idx].second;

    double map_yaw = 0.0;
    if (idx < path_points_.size() - 1) {
        double dx = path_points_[idx + 1].first - map_x;
        double dy = path_points_[idx + 1].second - map_y;
        map_yaw = std::atan2(dy, dx);
    } else if (idx > 0) {
        double dx = map_x - path_points_[idx - 1].first;
        double dy = map_y - path_points_[idx - 1].second;
        map_yaw = std::atan2(dy, dx);
    }

    double dx = map_x - front_x; 
    double dy = map_y - front_y;
    
    double error_front_axle = -std::sin(map_yaw) * dx + std::cos(map_yaw) * dy;

    double theta_e = map_yaw - yaw;
    while (theta_e > M_PI) theta_e -= 2.0 * M_PI;
    while (theta_e < -M_PI) theta_e += 2.0 * M_PI;

    double phi_d = std::atan2(k_gain_ * error_front_axle, v + k_soft_);
    double delta_raw = theta_e + phi_d;

    while (delta_raw > M_PI) delta_raw -= 2.0 * M_PI;
    while (delta_raw < -M_PI) delta_raw += 2.0 * M_PI;

    double delta_clamped = std::clamp(delta_raw, -max_steer_, max_steer_);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //     "Stanley Log | idx: %zu | CTE: %.3f | Theta_e: %.2f | Raw: %.1f deg -> Limit: %.1f",
    //     idx, error_front_axle, theta_e, rad2deg(delta_raw), rad2deg(delta_clamped));


    // Publish Cross-track Error
    std_msgs::msg::Float32 cte_msg;
    cte_msg.data = error_front_axle; // Biến e_fa
    error_cte_pub_->publish(cte_msg);

    // Publish Heading Error (đổi ra độ cho dễ nhìn)
    std_msgs::msg::Float32 head_msg;
    head_msg.data = rad2deg(theta_e);
    error_heading_pub_->publish(head_msg);


    return delta_clamped;
}

void StanleyController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (reached_goal_) {
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = this->now();
        stop_cmd.header.frame_id = "base_link";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        return; 
    }

    if (!has_path_ || path_points_.empty()) return;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    double v = desired_speed_; 

    double dx_g = x - path_points_.back().first;
    double dy_g = y - path_points_.back().second;
    double dist_to_goal = std::sqrt(dx_g*dx_g + dy_g*dy_g);

    if (dist_to_goal < goal_tolerance_ && current_index_ > (path_points_.size() * 0.9)) {
        reached_goal_ = true;
        RCLCPP_WARN(this->get_logger(), "!!! GOAL REACHED (Dist: %.2f) !!!", dist_to_goal);
        return;
    }

    double front_x = x + wheel_base_ * std::cos(yaw);
    double front_y = y + wheel_base_ * std::sin(yaw);

    double delta = computeSteering(front_x, front_y, yaw, v);
    double omega = (v / wheel_base_) * std::tan(delta);
    
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = v;
    cmd.twist.angular.z = omega;
    cmd_pub_->publish(cmd);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StanleyController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}