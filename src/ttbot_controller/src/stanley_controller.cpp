#include "ttbot_controller/stanley_controller.hpp"
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static double deg2rad(double deg) { return deg * M_PI / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

StanleyController::StanleyController()
: Node("stanley_controller")
{
    // --- KHAI BÁO PARAM ---
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("min_speed", 0.3);        // Mặc định 0.3 m/s khi cua
    this->declare_parameter("slow_down_factor", 4.0); // Hệ số giảm tốc
    
    this->declare_parameter("wheel_base", 0.65);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("k_gain", 2.0);  
    this->declare_parameter("k_soft", 1.0);  

    // --- LẤY GIÁ TRỊ PARAM ---
    desired_speed_    = this->get_parameter("desired_speed").as_double();
    min_speed_        = this->get_parameter("min_speed").as_double();
    slow_down_factor_ = this->get_parameter("slow_down_factor").as_double();
    
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    max_steer_     = deg2rad(this->get_parameter("max_steer_deg").as_double());
    k_gain_        = this->get_parameter("k_gain").as_double();
    k_soft_        = this->get_parameter("k_soft").as_double();
    goal_tolerance_= this->get_parameter("goal_tolerance").as_double();

    current_index_ = 0;
    has_path_ = false;
    reached_goal_ = false; 

    // --- INIT SUB/PUB ---
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
        "Stanley READY: MaxSteer=%.1f deg, V_max=%.2f, V_min=%.2f, K_gain=%.2f", 
        rad2deg(max_steer_), desired_speed_, min_speed_, k_gain_);
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
    RCLCPP_INFO(this->get_logger(), "--> PATH RECEIVED: %zu points.", path_points_.size());
}

size_t StanleyController::findClosestPoint(double front_x, double front_y)
{
    if (path_points_.empty()) return 0;

    size_t closest_idx = current_index_;
    double min_dist_sq = std::numeric_limits<double>::infinity();
    
    size_t search_window = 50; 
    
    size_t start_search = current_index_;
    size_t end_search = std::min(current_index_ + search_window, path_points_.size());

    if (current_index_ == 0) {
        end_search = std::min((size_t)100, path_points_.size());
    }

    for (size_t i = start_search; i < end_search; ++i) {
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

    // Tính hướng đường (Heading of path)
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

    // Tính sai số vị trí (Cross Track Error)
    double dx = map_x - front_x; 
    double dy = map_y - front_y;
    // Công thức vector đòn bẩy để tìm khoảng cách vuông góc có dấu
    double error_front_axle = -std::sin(map_yaw) * dx + std::cos(map_yaw) * dy;

    // Tính sai số hướng (Heading Error)
    double theta_e = map_yaw - yaw;
    while (theta_e > M_PI) theta_e -= 2.0 * M_PI;
    while (theta_e < -M_PI) theta_e += 2.0 * M_PI;

    // Công thức Stanley: delta = theta_e + atan(k * e / (v + k_soft))
    double phi_d = std::atan2(k_gain_ * error_front_axle, v + k_soft_);
    double delta_raw = theta_e + phi_d;

    // Chuẩn hóa góc lái
    while (delta_raw > M_PI) delta_raw -= 2.0 * M_PI;
    while (delta_raw < -M_PI) delta_raw += 2.0 * M_PI;
    double delta_clamped = std::clamp(delta_raw, -max_steer_, max_steer_);

    // Publish Debug info
    std_msgs::msg::Float32 cte_msg;
    cte_msg.data = error_front_axle;
    error_cte_pub_->publish(cte_msg);

    std_msgs::msg::Float32 head_msg;
    head_msg.data = rad2deg(theta_e);
    error_heading_pub_->publish(head_msg);

    return delta_clamped;
}

void StanleyController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Nếu đã đến đích thì dừng hẳn
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
    
    // --- KIỂM TRA ĐÍCH ĐẾN ---
    double dx_g = x - path_points_.back().first;
    double dy_g = y - path_points_.back().second;
    double dist_to_goal = std::sqrt(dx_g*dx_g + dy_g*dy_g);

    
    if (dist_to_goal < goal_tolerance_ && current_index_ > (path_points_.size() * 0.9)) {
        reached_goal_ = true;
        RCLCPP_WARN(this->get_logger(), "!!! GOAL REACHED (Dist: %.2f) !!!", dist_to_goal);
        return;
    }

    // --- TÍNH TOÁN STANLEY ---
    // Tính vị trí trục trước
    double front_x = x + wheel_base_ * std::cos(yaw);
    double front_y = y + wheel_base_ * std::sin(yaw);

    // B1: Tính góc lái cần thiết (delta) dựa trên vận tốc tham chiếu (để scaling gain không bị nhảy)
    double delta = computeSteering(front_x, front_y, yaw, desired_speed_);

    // B2: TÍNH VẬN TỐC ĐỘNG (DYNAMIC VELOCITY)
    // Công thức: v = v_min + (v_max - v_min) / (1 + factor * |angle|)
    double final_speed = min_speed_ + (desired_speed_ - min_speed_) / (1.0 + slow_down_factor_ * std::abs(delta));
    
    // Đảm bảo không thấp hơn min_speed (dù công thức trên đã lo, nhưng kẹp lại cho chắc)
    if (final_speed < min_speed_) final_speed = min_speed_;

    // B3: Tính Omega tương ứng với vận tốc mới
    double omega = (final_speed / wheel_base_) * std::tan(delta);
    
    // Publish
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = final_speed; // Dùng vận tốc động
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