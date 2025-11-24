#ifndef TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/utils.h"
#include <nav_msgs/msg/path.hpp>

#include <vector>
#include <utility>
#include <osqp/osqp.h>
#include <iostream>
#include <string>
#include <memory>

// Eigen headers
#include <Eigen/Sparse>
#include <Eigen/Dense>

// Định nghĩa trạng thái và điều khiển
using State   = std::vector<double>;  // [x, y, psi] (không còn dùng trong MPC mới)
using Control = std::vector<double>;  // [delta] (góc lái)

class MpcController : public rclcpp::Node
{
public:
    MpcController();
    ~MpcController();

private:
    // ==== Callbacks ====
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);


    // ==== Logic MPC ====
    Control solveMPC(double ey0, double epsi0);

    // Tuyến tính hóa mô hình cũ (hiện không dùng, nhưng vẫn để đó)
    void linearizeModel(double v, double psi, double dt);

    // Helper
    size_t findClosestPoint(double x, double y);

    void computeRefAtIndex(size_t idx,
                           double &rx, double &ry, double &psi_ref);

    void computeErrorState(double x, double y, double psi,
                           double rx, double ry, double psi_ref,
                           double &ey, double &epsi);

    void linearizeErrorModel(double v, double dt);

    void loadPathFromCSV();

    
    
    // Index hiện tại trên đường (khởi tạo = 0 trong constructor)
    size_t current_index_;

    // HÀM CHUYỂN ĐỔI Eigen -> OSQP CSC
    void eigenToOSQPCsc(
        const Eigen::SparseMatrix<double>& mat,
        OSQPCscMatrix& out_mat,  // Struct ma trận của OSQP
        OSQPFloat*& out_x,       // Data values
        OSQPInt*& out_i,         // Row indices
        OSQPInt*& out_p          // Column pointers
    );

    // ==== Tham số MPC / xe ====
    double desired_speed_; // m/s
    double wheel_base_;    // L
    double max_omega_;     // Giới hạn vận tốc góc (hiện không dùng trong QP)
    double max_steer_;     // Giới hạn góc lái (rad)

    // (cũ, không dùng nữa nhưng để lại nếu bạn chưa muốn xóa)
    double K_v_;
    double K_omega_;
    double Q_x_;
    double Q_psi_;
    double R_omega_;

    // MPC params (mới)
    int    N_p_;
    double dt_mpc_;
    double Q_ey_;
    double Q_epsi_;
    double R_delta_;

    // ==== Goal Parameters ====
    double goal_tolerance_;
    bool reached_goal_;

    // ==== Path Parameters ====
    bool has_path_ = false;


    // ==== Tham số Tối ưu hóa (nếu sau này cần) ====
    // int N_c_; // Control Horizon

    // ==== Mô hình Tuyến tính hóa cũ (không còn dùng trong QP) ====
    std::vector<double> A_flat_, B_flat_;

    // ==== Path data ====
    std::vector<std::pair<double, double>> path_points_;
    std::string path_file_;

    // Ma trận tuyến tính hóa error model (2x2, 2x1)
    Eigen::Matrix2d Ad_;
    Eigen::Vector2d Bd_;

    // ==== ROS ====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // ==== OSQP Solver Data (v1.0+) ====
    OSQPSolver*   solver_   = nullptr;
    OSQPSettings* settings_ = nullptr;

    // Dữ liệu cho Ma trận P (Cost Matrix)
    OSQPFloat* P_x_ = nullptr;
    OSQPInt*   P_i_ = nullptr;
    OSQPInt*   P_p_ = nullptr;

    // Dữ liệu cho Ma trận A (Constraint Matrix)
    OSQPFloat* A_x_ = nullptr;
    OSQPInt*   A_i_ = nullptr;
    OSQPInt*   A_p_ = nullptr;

    // Dữ liệu cho các Vector
    OSQPFloat* q_data_ = nullptr; // Vector q (linear cost)
    OSQPFloat* l_data_ = nullptr; // Vector lower bound
    OSQPFloat* u_data_ = nullptr; // Vector upper bound
};

#endif // TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_
