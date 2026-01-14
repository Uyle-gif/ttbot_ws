#include "ttbot_controller/mpc_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <cmath>

// Helper functions
static double deg2rad(double deg) { return deg * M_PI / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

MpcController::MpcController()
: Node("mpc_controller")
{
    // 1. Declare & Load Parameters
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("wheel_base", 0.65);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("goal_tolerance", 0.3);

    // MPC Weights
    this->declare_parameter("N_p", 10);
    this->declare_parameter("dt_mpc", 0.1);
    this->declare_parameter("Q_ey", 10.0);
    this->declare_parameter("Q_epsi", 5.0);
    this->declare_parameter("R_delta", 1.0);

    desired_speed_ = this->get_parameter("desired_speed").as_double();
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
    
    N_p_    = this->get_parameter("N_p").as_int();
    dt_mpc_ = this->get_parameter("dt_mpc").as_double();
    Q_ey_   = this->get_parameter("Q_ey").as_double();
    Q_epsi_ = this->get_parameter("Q_epsi").as_double();
    R_delta_= this->get_parameter("R_delta").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    // Constraints
    max_steer_ = deg2rad(max_steer_deg);
    // Tính max omega dựa trên mô hình Ackermann: w = (v/L) * tan(delta)
    max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);

    // Init State
    current_index_ = 0;
    has_path_ = false;
    reached_goal_ = false;

    // 2. ROS setup
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&MpcController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mpc_path", 1,
        std::bind(&MpcController::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann_controller/cmd_vel", 10);
        
    // --- KHỞI TẠO PUBLISHER ĐỂ VẼ ĐỒ THỊ ---
    error_cte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/mpc/error/cte", 10);
    error_heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/mpc/error/heading", 10);
    // -------------------------------------

    RCLCPP_INFO(this->get_logger(), 
        "MPC Initialized: N=%d, dt=%.2f, Speed=%.2f, MaxSteer=%.1f deg", 
        N_p_, dt_mpc_, desired_speed_, max_steer_deg);
}

MpcController::~MpcController()
{
    freeOSQPMemory();
}

void MpcController::freeOSQPMemory()
{
    // Clean up Solver
    if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }
    if (settings_) { free(settings_); settings_ = nullptr; }

    // Clean up Data Arrays
    if (P_x_) { free(P_x_); P_x_ = nullptr; }
    if (P_i_) { free(P_i_); P_i_ = nullptr; }
    if (P_p_) { free(P_p_); P_p_ = nullptr; }

    if (A_x_) { free(A_x_); A_x_ = nullptr; }
    if (A_i_) { free(A_i_); A_i_ = nullptr; }
    if (A_p_) { free(A_p_); A_p_ = nullptr; }

    if (q_data_) { free(q_data_); q_data_ = nullptr; }
    if (l_data_) { free(l_data_); l_data_ = nullptr; }
    if (u_data_) { free(u_data_); u_data_ = nullptr; }
}

void MpcController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) return;

    path_points_.clear();
    path_points_.reserve(msg->poses.size());

    for (const auto &pose : msg->poses) {
        path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    current_index_ = 0;
    has_path_ = true;
    reached_goal_ = false; // Reset trạng thái khi có path mới

    RCLCPP_INFO(this->get_logger(), "--> NEW PATH: %zu points. MPC Ready.", path_points_.size());
}

size_t MpcController::findClosestPoint(double x, double y)
{
    if (path_points_.empty()) return 0;

    size_t best_idx = current_index_;
    double min_dist_sq = std::numeric_limits<double>::infinity();

    // Tối ưu: Chỉ tìm kiếm trong cửa sổ +100 điểm phía trước
    size_t search_end = std::min(current_index_ + 100, path_points_.size());
    if (current_index_ == 0) search_end = path_points_.size(); 

    for (size_t i = current_index_; i < search_end; ++i) {
        double dx = x - path_points_[i].first;
        double dy = y - path_points_[i].second;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    current_index_ = best_idx;
    return best_idx;
}

void MpcController::computeReference(size_t idx, double &rx, double &ry, double &rpsi)
{
    rx = path_points_[idx].first;
    ry = path_points_[idx].second;

    // Tính psi_ref từ điểm tiếp theo
    if (idx + 1 < path_points_.size()) {
        double dx = path_points_[idx+1].first - rx;
        double dy = path_points_[idx+1].second - ry;
        rpsi = std::atan2(dy, dx);
    } else if (idx > 0) {
        double dx = rx - path_points_[idx-1].first;
        double dy = ry - path_points_[idx-1].second;
        rpsi = std::atan2(dy, dx);
    } else {
        rpsi = 0.0;
    }
}

void MpcController::computeErrorState(double x, double y, double yaw,
                                      double rx, double ry, double rpsi,
                                      double &ey, double &epsi)
{
    double dx = x - rx;
    double dy = y - ry;

    // Lỗi ngang e_y
    ey = -std::sin(rpsi) * dx + std::cos(rpsi) * dy;

    // Lỗi góc e_psi
    epsi = yaw - rpsi;
    while (epsi > M_PI) epsi -= 2.0*M_PI;
    while (epsi < -M_PI) epsi += 2.0*M_PI;
}

void MpcController::linearizeErrorModel(double v, double dt)
{
    Eigen::Matrix2d A;
    A << 1.0, v * dt,
         0.0, 1.0;

    Eigen::Vector2d B;
    B << 0.0,
         (v * dt) / wheel_base_;

    Ad_ = A;
    Bd_ = B;
}

void MpcController::eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                                   OSQPCscMatrix& out_mat,
                                   OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p)
{
    OSQPInt nnz = mat.nonZeros();
    out_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * nnz);
    out_i = (OSQPInt*)malloc(sizeof(OSQPInt) * nnz);
    out_p = (OSQPInt*)malloc(sizeof(OSQPInt) * (mat.cols() + 1));

    for (int k = 0; k < nnz; k++) {
        out_x[k] = (OSQPFloat)mat.valuePtr()[k];
        out_i[k] = (OSQPInt)mat.innerIndexPtr()[k];
    }
    for (int k = 0; k < mat.cols() + 1; k++) {
        out_p[k] = (OSQPInt)mat.outerIndexPtr()[k];
    }

    out_mat.m = mat.rows();
    out_mat.n = mat.cols();
    out_mat.nzmax = nnz;
    out_mat.nz = -1;
    out_mat.owned = 0; 
    out_mat.x = out_x;
    out_mat.i = out_i;
    out_mat.p = out_p;
}

Control MpcController::solveMPC(double ey0, double epsi0, double v_ref)
{
    freeOSQPMemory();

    int nx = 2; // [ey, epsi]
    int nu = 1; // [delta]
    int N  = N_p_;
    
    int n_vars = (N + 1) * nx + N * nu;
    int n_eq   = (N + 1) * nx; 
    int n_ineq = N * nu;
    int n_cons = n_eq + n_ineq;

    linearizeErrorModel(v_ref, dt_mpc_);

    // --- Build Cost Matrix P ---
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;
    
    for (int k = 0; k <= N; ++k) {
        int offset = k * nx;
        p_triplets.emplace_back(offset,     offset,     Q_ey_);
        p_triplets.emplace_back(offset + 1, offset + 1, Q_epsi_);
    }
    int u_start_idx = (N + 1) * nx;
    for (int k = 0; k < N; ++k) {
        int offset = u_start_idx + k * nu;
        p_triplets.emplace_back(offset, offset, R_delta_);
    }
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // --- Build Constraints Matrix A ---
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_cons);

    // 1. Init state constraint
    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
        l(i) = (i == 0) ? ey0 : epsi0;
        u(i) = (i == 0) ? ey0 : epsi0;
    }

    // 2. Dynamics constraints
    for (int k = 0; k < N; ++k) {
        int row = nx + k * nx;
        int xk  = k * nx;
        int xk1 = (k + 1) * nx;
        int uk  = u_start_idx + k * nu;

        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                if (std::abs(Ad_(r,c)) > 1e-5)
                    a_triplets.emplace_back(row + r, xk + c, -Ad_(r,c));
            }
        }
        for (int r = 0; r < nx; ++r) {
            a_triplets.emplace_back(row + r, xk1 + r, 1.0);
        }
        for (int r = 0; r < nx; ++r) {
            if (std::abs(Bd_(r)) > 1e-5)
                a_triplets.emplace_back(row + r, uk, -Bd_(r));
        }
        l(row) = 0.0; u(row) = 0.0;
        l(row+1) = 0.0; u(row+1) = 0.0;
    }

    // 3. Input constraints
    int ineq_start = n_eq;
    for (int k = 0; k < N; ++k) {
        int row = ineq_start + k;
        int col = u_start_idx + k * nu;
        a_triplets.emplace_back(row, col, 1.0);
        l(row) = -max_steer_;
        u(row) =  max_steer_;
    }

    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());

    OSQPCscMatrix P_mat, A_mat;
    eigenToOSQPCsc(P, P_mat, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_cons, A_mat, A_x_, A_i_, A_p_);

    q_data_ = (OSQPFloat*)calloc(n_vars, sizeof(OSQPFloat));
    
    l_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    u_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    for(int i=0; i<n_cons; ++i){
        l_data_[i] = (OSQPFloat)l(i);
        u_data_[i] = (OSQPFloat)u(i);
    }

    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;

    osqp_setup(&solver_, &P_mat, q_data_, &A_mat, l_data_, u_data_, n_cons, n_vars, settings_);
    osqp_solve(solver_);

    double delta_opt = 0.0;
    if (solver_->info->status_val == OSQP_SOLVED) {
        delta_opt = solver_->solution->x[u_start_idx];
    } else {
        RCLCPP_WARN(get_logger(), "MPC Unsolved: %s", solver_->info->status);
    }

    return {delta_opt};
}

void MpcController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // ==========================================
    // 1. FIREWALL: Dừng xe tuyệt đối nếu đã tới đích
    // ==========================================
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
    double v_ref = desired_speed_;

    // ==========================================
    // 2. CHECK GOAL & IMMEDIATE STOP
    // ==========================================
    double dx_g = x - path_points_.back().first;
    double dy_g = y - path_points_.back().second;
    double dist_to_goal = std::sqrt(dx_g*dx_g + dy_g*dy_g);

    bool near_end = current_index_ > (path_points_.size() * 0.9);
    if (dist_to_goal < goal_tolerance_ && near_end) {
        reached_goal_ = true;
        RCLCPP_WARN(this->get_logger(), "!!! MPC GOAL REACHED (%.2fm) - STOPPING !!!", dist_to_goal);
        
        // --- GỬI LỆNH DỪNG NGAY LẬP TỨC ---
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = this->now();
        stop_cmd.header.frame_id = "base_link";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        // ----------------------------------
        return; 
    }

    // ==========================================
    // 3. CALCULATION
    // ==========================================
    size_t idx = findClosestPoint(x, y);
    double rx, ry, rpsi;
    computeReference(idx, rx, ry, rpsi);

    double ey, epsi;
    computeErrorState(x, y, yaw, rx, ry, rpsi, ey, epsi);

    // --- PUBLISH SAI SỐ ĐỂ VẼ ĐỒ THỊ ---
    std_msgs::msg::Float32 cte_msg;
    cte_msg.data = ey; 
    error_cte_pub_->publish(cte_msg);

    std_msgs::msg::Float32 head_msg;
    head_msg.data = rad2deg(epsi); // Đổi sang độ cho dễ đọc
    error_heading_pub_->publish(head_msg);
    // ----------------------------------

    Control u = solveMPC(ey, epsi, v_ref);
    double delta = u[0];

    double omega = (v_ref / wheel_base_) * std::tan(delta);
    omega = std::clamp(omega, -max_omega_, max_omega_);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //     "MPC LOG | idx: %zu | ey: %.3f | epsi: %.3f | Delta: %.1f deg",
    //     idx, ey, epsi, rad2deg(delta));

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = v_ref;
    cmd.twist.angular.z = omega;
    cmd_pub_->publish(cmd);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}