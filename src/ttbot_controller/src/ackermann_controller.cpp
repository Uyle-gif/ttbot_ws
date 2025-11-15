#include "ttbot_controller/ackermann_controller.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

AckermannController::AckermannController(const std::string & name)
: Node(name)
{
    declare_parameter("wheel_radius", 0.15);
    declare_parameter("wheel_base", 0.8);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_base_   = get_parameter("wheel_base").as_double();

    RCLCPP_INFO(get_logger(), "Using wheel_radius = %f", wheel_radius_);
    RCLCPP_INFO(get_logger(), "Using wheel_base   = %f", wheel_base_);
    
    // Publisher tới controller
    rear_wheel_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "/rear_wheel_velocity_controller/commands", 10);

    steering_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "/front_steering_position_controller/commands", 10);

    // Sub nhận /cmd_vel
    vel_sub_ =
        create_subscription<geometry_msgs::msg::TwistStamped>(
            "/ackermann_controller/cmd_vel", 10,
            std::bind(&AckermannController::velCallback, this, std::placeholders::_1)
        );


    // Sub /joint_states để đọc encoder
    joint_sub_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",                
            10,
            std::bind(&AckermannController::jointCallback, this, std::placeholders::_1)
        );

    // Publisher odometry
    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(
            "/ackermann_controller/odom", 10);


    // Khởi tạo message odometry
    odom_msg_.header.frame_id       = "odom";
    odom_msg_.child_frame_id        = "base_link";
    odom_msg_.pose.pose.orientation.x = 1.0; // mặc định không quay
    odom_msg_.pose.pose.orientation.y = 1.0; // mặc định không quay
    odom_msg_.pose.pose.orientation.z = 1.0; // mặc định không quay
    odom_msg_.pose.pose.orientation.w = 1.0; // mặc định không quay

    // TF broadcaster
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";


    // ====== INIT ======
    rear_left_prev_pos_  = 0.0;
    rear_right_prev_pos_ = 0.0;
    prev_time_           = this->get_clock()->now();


    // ====== INIT POSE ODOMETRY ======
    x_     = 0.0;
    y_     = 0.0;
    theta_ = 0.0;






}

void AckermannController::velCallback(const geometry_msgs::msg::TwistStamped & msg)
{
    double v = msg.twist.linear.x;
    double w = msg.twist.angular.z;

    double steering_angle = 0.0;
    if (std::abs(v) > 1e-4)
        steering_angle = std::atan(w * wheel_base_ / v);

    // tốc độ bánh sau (rad/s)
    double wheel_speed = v / wheel_radius_;

    // gửi tốc độ bánh sau *cho 2 bánh*
    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data.resize(2);
    wheel_msg.data[0] = wheel_speed;  // rear_left_wheel_joint
    wheel_msg.data[1] = wheel_speed;  // rear_right_wheel_joint
    rear_wheel_pub_->publish(wheel_msg);

    // gửi góc lái *cho 2 bánh trước*
    std_msgs::msg::Float64MultiArray steer_msg;
    steer_msg.data.resize(2);
    steer_msg.data[0] = steering_angle;   // front_left_steering_joint
    steer_msg.data[1] = steering_angle;   // front_right_steering_joint
    steering_pub_->publish(steer_msg);
    }



void AckermannController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    // tìm index các joint trong msg.name
    int idx_rl = -1, idx_rr = -1;
    int idx_fl_steer = -1, idx_fr_steer = -1;

    for (size_t i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == "rear_left_wheel_joint")
            idx_rl = static_cast<int>(i);
        else if (msg.name[i] == "rear_right_wheel_joint")
            idx_rr = static_cast<int>(i);
        else if (msg.name[i] == "front_left_steering_joint")
            idx_fl_steer = static_cast<int>(i);
        else if (msg.name[i] == "front_right_steering_joint")
            idx_fr_steer = static_cast<int>(i);
    }

    if (idx_rl < 0 || idx_rr < 0 || idx_fl_steer < 0 || idx_fr_steer < 0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "jointCallback: không tìm được đủ joint trong JointState");
        return;
    }

    // =====tính Δt ====
    rclcpp::Time msg_time(msg.header.stamp);
    rclcpp::Duration d = msg_time - prev_time_;
    double dt = d.seconds();

    if (dt <= 0.0) {
        prev_time_ = msg_time;
        return;
    }

    // ====  tính Δposition cho 2 bánh sau ====
    double pos_rl = msg.position[idx_rl];
    double pos_rr = msg.position[idx_rr];

    double dp_rl = pos_rl - rear_left_prev_pos_;
    double dp_rr = pos_rr - rear_right_prev_pos_;

    // ==== update biến hỗ trợ cho lần sau ====
    rear_left_prev_pos_  = pos_rl;
    rear_right_prev_pos_ = pos_rr;
    prev_time_           = msg_time;

    // ==== tính tốc độ quay bánh (rad/s) ====
    double phi_rl = dp_rl / dt;
    double phi_rr = dp_rr / dt;

    // ==== từ đó tính v robot (trên trục sau) ====
    double v = wheel_radius_ * 0.5 * (phi_rl + phi_rr);   // m/s

    // ==== lấy góc lái hiện tại (trung bình 2 bánh trước) ====
    double delta_fl = msg.position[idx_fl_steer];
    double delta_fr = msg.position[idx_fr_steer];
    double steering_angle = 0.5 * (delta_fl + delta_fr);  // rad

    // ==== Ackermann kinematics: w = v * tan(delta) / L ====
    double w = 0.0;
    if (std::abs(steering_angle) > 1e-4) {
        w = v * std::tan(steering_angle) / wheel_base_;
    }

    // ====== TÍCH PHÂN ODOMETRY (mô hình xe đạp)  ======
    // x' = v cos(theta), y' = v sin(theta), theta' = w
    // dùng mid-point rule để tích phân
    double theta_mid = theta_ + 0.5 * w * dt;

    x_     += v * std::cos(theta_mid) * dt;
    y_     += v * std::sin(theta_mid) * dt;
    theta_ += w * dt;

    // chuẩn hóa góc về [-pi, pi]
    while (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;


    // điền message odometry
    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;;
    odom_msg_.pose.pose.position.y = y_;;
    odom_msg_.twist.twist.linear.x = v;
    odom_msg_.twist.twist.angular.z = w;


    // điền TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = get_clock()->now();
    


    // publish odometry
    odom_pub_->publish(odom_msg_);    
    // publish TF
    transform_broadcaster_->sendTransform(transform_stamped_);


    // In thông tin ra console
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,   
        "Ackermann: v = %.3f [m/s], w = %.3f [rad/s], "
        "x = %.2f [m], y = %.2f [m], th = %.2f [rad]",
        v, w, x_, y_, theta_);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannController>("ackermann_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
