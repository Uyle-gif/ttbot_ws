#ifndef TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp" // <--- [MỚI] Thêm thư viện này
#include "tf2/utils.h"

#include <vector>
#include <utility>
#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

// Eigen & OSQP
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <osqp/osqp.h>

using Control = std::vector<double>; // [delta]

class MpcController : public rclcpp::Node
{
public:
    MpcController();
    ~MpcController();

private:
    // ==== Callbacks ====
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // ==== MPC Logic ====
    Control solveMPC(double ey0, double epsi0, double v_ref);
    
    void linearizeErrorModel(double v, double dt);
    
    // Helper tìm điểm và tính lỗi
    size_t findClosestPoint(double x, double y);
    void computeReference(size_t idx, double &rx, double &ry, double &rpsi);
    void computeErrorState(double x, double y, double yaw,
                           double rx, double ry, double rpsi,
                           double &ey, double &epsi);

    // Helper quản lý bộ nhớ OSQP (Tránh memory leak)
    void freeOSQPMemory();
    void eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                        OSQPCscMatrix& out_mat,
                        OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p);

    // ==== Parameters ====
    double desired_speed_;
    double wheel_base_;
    double max_steer_;     // rad
    double max_omega_;     // rad/s (tính từ max_steer)

    // Weights (Trọng số)
    int    N_p_;       // Horizon
    double dt_mpc_;    // Step time
    double Q_ey_;
    double Q_epsi_;
    double R_delta_;

    // Goal
    double goal_tolerance_;
    bool reached_goal_;

    // Path
    std::vector<std::pair<double, double>> path_points_;
    size_t current_index_;
    bool has_path_;

    // ==== Model Matrices (Linearized) ====
    Eigen::Matrix2d Ad_;
    Eigen::Vector2d Bd_;

    // ==== OSQP Data Pointers ====
    // Lưu ý: Cần giữ lại để free sau mỗi vòng lặp
    OSQPSolver* solver_   = nullptr;
    OSQPSettings* settings_ = nullptr;
    
    OSQPFloat* P_x_ = nullptr; OSQPInt* P_i_ = nullptr; OSQPInt* P_p_ = nullptr;
    OSQPFloat* A_x_ = nullptr; OSQPInt* A_i_ = nullptr; OSQPInt* A_p_ = nullptr;
    OSQPFloat* q_data_ = nullptr;
    OSQPFloat* l_data_ = nullptr;
    OSQPFloat* u_data_ = nullptr;

    // ==== ROS ====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    // [MỚI] Thêm 2 Publisher để vẽ đồ thị đánh giá
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_cte_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_heading_pub_;
};

#endif // TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_