#ifndef TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/utils.h"
#include <vector>
#include <utility>
#include <osqp/osqp.h>
#include <iostream>
#include <string> // Thêm string
#include <memory> // Thêm memory cho smart pointers
// Eigen headers
#include <Eigen/Sparse>
#include <Eigen/Dense>
// Định nghĩa trạng thái và điều khiển
using State = std::vector<double>; // [x, y, psi]
using Control = std::vector<double>; // [omega]

class MpcController : public rclcpp::Node
{
public:
    MpcController();
    ~MpcController();
private:
    // ==== Callbacks ====
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ==== Logic MPC ====
    Control solveMPC(const State& current_state);
    void linearizeModel(double v, double psi, double dt); // Tuyến tính hóa mô hình
    void loadPathFromCSV(); // Tái sử dụng hàm tải path
    size_t findClosestPoint(double x, double y); // <-- THÊM VÀO ĐÂY
    // HÀM CHUYỂN ĐỔI (Cần khai báo)
    void eigenToOSQPCsc(
        const Eigen::SparseMatrix<double>& mat,
        OSQPCscMatrix& out_mat,  // Struct ma trận của OSQP
        OSQPFloat*& out_x,       // Data values
        OSQPInt*& out_i,         // Row indices
        OSQPInt*& out_p          // Column pointers
    );

    // ==== Tham số MPC ====
    double K_v_;           // Trọng số vận tốc (Q matrix)
    double K_omega_;       // Trọng số điều khiển (R matrix)
    double desired_speed_; // m/s
    double wheel_base_;    // L
    double max_omega_;     // Giới hạn vận tốc góc
    double Q_x_;
    double Q_psi_;
    double R_omega_;
    double max_steer_ ;


    // ==== Tham số Tối ưu hóa ====
    int N_p_; // Prediction Horizon
    int N_c_; // Control Horizon
    double dt_mpc_; // Bước thời gian rời rạc

    // ==== Mô hình Tuyến tính hóa (Ma trận A và B) ====
    std::vector<double> A_flat_, B_flat_; // Lưu ma trận A, B dưới dạng phẳng

    // ==== Path data ====
    std::vector<std::pair<double, double>> path_points_;
    std::string path_file_;

    // ==== ROS ====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    // ==== OSQP Solver Data (Phiên bản v1.0+) ====
    // Solver chính
    OSQPSolver* solver_ = nullptr;
    OSQPSettings* settings_ = nullptr;

    // Các con trỏ quản lý bộ nhớ cho Ma trận và Vector (Raw Data)
    // Cần giữ các biến này tồn tại trong class để quản lý malloc/free
    
    // Dữ liệu cho Ma trận P (Cost Matrix)
    OSQPFloat* P_x_ = nullptr;
    OSQPInt* P_i_ = nullptr;
    OSQPInt* P_p_ = nullptr;

    // Dữ liệu cho Ma trận A (Constraint Matrix)
    OSQPFloat* A_x_ = nullptr;
    OSQPInt* A_i_ = nullptr;
    OSQPInt* A_p_ = nullptr;

    // Dữ liệu cho các Vector
    OSQPFloat* q_data_ = nullptr; // Vector q (linear cost)
    OSQPFloat* l_data_ = nullptr; // Vector lower bound
    OSQPFloat* u_data_ = nullptr; // Vector upper bound
    
};

#endif // TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_