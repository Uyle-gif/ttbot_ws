#include "ttbot_controller/mpc_controller.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>
#include <Eigen/Dense> // Sử dụng Eigen cho tính toán ma trận
#include <osqp/osqp.h> // Thư viện giải QP
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <utility>
#include <string>
#include <iostream>
#include <vector>
#include <cstdlib> // <--- THÊM DÒNG NÀY để dùng malloc/free
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// Định nghĩa không gian tên và các hằng số
namespace {
constexpr const char* kPackageName = "ttbot_controller";

} // namespace

// Khởi tạo MPC Controller
MpcController::MpcController()
: Node("mpc_controller")
{
    // 1. Khai báo và Load Parameters (Dùng lại của Stanley và thêm MPC)
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("wheel_base", 0.8);
    this->declare_parameter("max_steer_deg", 60.0);
    this->declare_parameter("path_file", "path.csv");
    this->declare_parameter("goal_tolerance", 0.3);  // [m] bán kính để coi là tới đích



    // MPC Parameters
    this->declare_parameter("N_p", 10);
    this->declare_parameter("dt_mpc", 0.1);
    this->declare_parameter("Q_ey", 10.0);
    this->declare_parameter("Q_epsi", 5.0);
    this->declare_parameter("R_delta", 1.0);


    // Load
    desired_speed_ = this->get_parameter("desired_speed").as_double();
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
    // path_file_     = this->get_parameter("path_file").as_string();

    N_p_    = this->get_parameter("N_p").as_int();
    dt_mpc_ = this->get_parameter("dt_mpc").as_double();
    Q_ey_   = this->get_parameter("Q_ey").as_double();
    Q_epsi_ = this->get_parameter("Q_epsi").as_double();
    R_delta_= this->get_parameter("R_delta").as_double();

    // Mục tiêu đến đích
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    reached_goal_ = false;  // xe chưa tới đích

    // Ràng buộc điều khiển
    max_steer_ = max_steer_deg * M_PI / 180.0;
    max_omega_ = desired_speed_ * std::tan(max_steer_) / wheel_base_; 
    
    // Khởi tạo con trỏ null (Quan trọng để tránh lỗi destructor lần đầu)
    solver_ = nullptr;
    settings_ = nullptr;
    P_x_ = nullptr; P_i_ = nullptr; P_p_ = nullptr;
    A_x_ = nullptr; A_i_ = nullptr; A_p_ = nullptr;
    q_data_ = nullptr; l_data_ = nullptr; u_data_ = nullptr;
    // 2. Load Path
    // Tái sử dụng hàm loadPathFromCSV() đã có (cần sao chép vào tệp này hoặc khai báo bên ngoài)


   // loadPathFromCSV(); 


    current_index_ = 0;
    has_path_ = false;



    // 3. Subscribers & Publishers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&MpcController::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_controller/cmd_vel", 10);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mpc_path", 1,
        std::bind(&MpcController::pathCallback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "MPC Controller initialized with N_p=%d, dt=%.2f", N_p_, dt_mpc_);
}


// =================================================================================
// Path Callback: Nhận đường đi từ bên ngoài
// =================================================================================
void MpcController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_points_.clear();
    path_points_.reserve(msg->poses.size());

    for (const auto &pose_stamped : msg->poses) {
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        path_points_.emplace_back(x, y);
    }

    current_index_ = 0;
    has_path_ = !path_points_.empty();

    RCLCPP_INFO(this->get_logger(),
                "Received /mpc_path with %zu points", path_points_.size());
}




// =================================================================================
// Destructor: Dọn dẹp bộ nhớ thủ công (Bắt buộc với OSQP)
// =================================================================================

void MpcController::computeRefAtIndex(size_t idx,
                                      double &rx, double &ry, double &psi_ref)
{
    idx = std::min(idx, path_points_.size() - 1);
    rx  = path_points_[idx].first;
    ry  = path_points_[idx].second;

    // Tính hướng của path tại idx từ điểm lân cận
    size_t idx2 = std::min(idx + 1, path_points_.size() - 1);
    if (idx2 == idx && idx > 0) {
        idx2 = idx - 1;
    }

    double dx = path_points_[idx2].first  - rx;
    double dy = path_points_[idx2].second - ry;
    psi_ref   = std::atan2(dy, dx);
}

void MpcController::computeErrorState(double x, double y, double psi,
                                      double rx, double ry, double psi_ref,
                                      double &ey, double &epsi)
{
    // Vector từ điểm ref đến xe
    double dx = x - rx;
    double dy = y - ry;

    // Lỗi ngang e_y: chiếu lên trục vuông góc với path
    // trục path: (cos(psi_ref), sin(psi_ref))
    // trục ngang trái: (-sin, cos)
    ey = -std::sin(psi_ref) * dx + std::cos(psi_ref) * dy;

    // Lỗi góc
    double dpsi = psi - psi_ref;
    while (dpsi > M_PI)  dpsi -= 2.0*M_PI;
    while (dpsi < -M_PI) dpsi += 2.0*M_PI;
    epsi = dpsi;
}
void MpcController::linearizeErrorModel(double v, double dt)
{
    Eigen::Matrix2d A_cont;
    Eigen::Vector2d B_cont;

    A_cont << 0.0, v,
              0.0, 0.0;

    B_cont << 0.0,
              v / wheel_base_;

    Ad_ = Eigen::Matrix2d::Identity() + A_cont * dt;
    Bd_ = B_cont * dt;
}


MpcController::~MpcController()
{
    // Dọn dẹp solver
    if (solver_) {
        osqp_cleanup(solver_);
    }
    if (settings_) {
        free(settings_); // Thay c_free bằng free
    }

    // Dọn dẹp các mảng dữ liệu thô (Tách dòng để tránh warning indentation)
    if (P_x_) free(P_x_); 
    if (P_i_) free(P_i_); 
    if (P_p_) free(P_p_);
    
    if (A_x_) free(A_x_); 
    if (A_i_) free(A_i_); 
    if (A_p_) free(A_p_);
    
    if (q_data_) free(q_data_);
    if (l_data_) free(l_data_);
    if (u_data_) free(u_data_);
}
// Chú ý: Hàm loadPathFromCSV() cần được sao chép/triển khai tương tự Stanley Controller
// ================================
// Load CSV Path File
// ================================
void MpcController::loadPathFromCSV()
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


void MpcController::linearizeModel(double v, double psi, double dt)
{
    // Khai báo ma trận trạng thái (3x3) và điều khiển (3x1)
    Eigen::Matrix3d A_cont; // Ma trận A liên tục
    Eigen::Vector3d B_cont; // Vector B liên tục

    // A = [0 0 -v*sin(psi); 0 0 v*cos(psi); 0 0 0]
    A_cont << 
        0, 0, -v * std::sin(psi),
        0, 0, v * std::cos(psi),
        0, 0, 0;

    // B = [0; 0; 1]
    B_cont << 
        0,
        0,
        1;

    // Rời rạc hóa (Euler Forward): Ad = I + A*dt, Bd = B*dt
    Eigen::Matrix3d A_disc = Eigen::Matrix3d::Identity() + A_cont * dt;
    Eigen::Vector3d B_disc = B_cont * dt;

    // Lưu kết quả dưới dạng phẳng (flat) để truyền vào OSQP
    // OSQP yêu cầu ma trận/vector dưới dạng C-style array
    A_flat_.clear();
    B_flat_.clear();
    
    // Lưu A_disc (3x3)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            A_flat_.push_back(A_disc(i, j));
        }
    }
    // Lưu B_disc (3x1)
    for (int i = 0; i < 3; ++i) {
        B_flat_.push_back(B_disc(i));
    }
}

// ================================
// Helper: Find closest point on path
// ================================
size_t MpcController::findClosestPoint(double x, double y)
{
  if (path_points_.empty()) {
    return 0;
  }

  // Chỉ tìm từ current_index_ trở về sau
  size_t start_i = current_index_;
  size_t closest_index = start_i;
  double min_dist_sq   = std::numeric_limits<double>::infinity();

  for (size_t i = start_i; i < path_points_.size(); ++i) {
    double dx = x - path_points_[i].first;
    double dy = y - path_points_[i].second;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_index = i;
    }
  }

  // Cập nhật luôn index hiện tại để lần sau không lùi lại
  current_index_ = closest_index;
  return closest_index;
}

// =================================================================================
// Helper: Eigen to OSQP Conversion
// =================================================================================
void MpcController::eigenToOSQPCsc(
    const Eigen::SparseMatrix<double>& mat, 
    OSQPCscMatrix& out_mat, 
    OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p)
{
    if (!mat.isCompressed()) return;

    OSQPInt nnz  = mat.nonZeros();
    OSQPInt cols = mat.cols();
    OSQPInt rows = mat.rows();

    // KHÔNG free ở đây nữa, chỉ cấp phát mới
    out_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * nnz);
    out_i = (OSQPInt*)malloc(sizeof(OSQPInt) * nnz);
    out_p = (OSQPInt*)malloc(sizeof(OSQPInt) * (cols + 1));

    for (int k = 0; k < nnz; k++) {
        out_x[k] = (OSQPFloat)mat.valuePtr()[k];
        out_i[k] = (OSQPInt)mat.innerIndexPtr()[k];
    }
    for (int k = 0; k < cols + 1; k++) {
        out_p[k] = (OSQPInt)mat.outerIndexPtr()[k];
    }

    out_mat.m = rows;
    out_mat.n = cols;
    out_mat.p = out_p;
    out_mat.i = out_i;
    out_mat.x = out_x;
    out_mat.nzmax = nnz;
    out_mat.nz = -1;
    out_mat.owned = 0;
}

// =================================================================================
// SOLVE MPC (CORE LOGIC)
// =================================================================================
Control MpcController::solveMPC(double ey0, double epsi0)
{
    // --- 1. Cấu hình kích thước ---
    int nx = 2; // [ey, epsi]
    int nu = 1; // delta (góc lái)
    int N  = N_p_;

    int n_vars = (N + 1) * nx + N * nu;
    int n_eq   = (N + 1) * nx; // x0 + N bước động lực học
    int n_ineq = N * nu;       // giới hạn delta
    int n_cons = n_eq + n_ineq;

    // --- 2. Tuyến tính hóa mô hình lỗi ---
    linearizeErrorModel(desired_speed_, dt_mpc_);
    Eigen::Matrix2d Ad = Ad_;
    Eigen::Vector2d Bd = Bd_;

    // --- 3. Cost: P, q (q = 0 vì ref = 0 cho ey, epsi) ---
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

    // State cost
    for (int k = 0; k <= N; ++k) {
        int offset = k * nx;
        p_triplets.emplace_back(offset + 0, offset + 0, Q_ey_);   // ey
        p_triplets.emplace_back(offset + 1, offset + 1, Q_epsi_); // epsi
    }

    // Input cost
    int u_start = (N + 1) * nx;
    for (int k = 0; k < N; ++k) {
        int offset = u_start + k * nu;
        p_triplets.emplace_back(offset, offset, R_delta_);
    }
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // --- 4. Constraints A, l, u ---
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_cons);

    // 4a. Ràng buộc trạng thái đầu: x0 = [ey0, epsi0]
    Eigen::Vector2d x0;
    x0 << ey0, epsi0;
    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
        l(i) = x0(i);
        u(i) = x0(i);
    }

    // 4b. Ràng buộc động lực học: x_{k+1} = Ad*x_k + Bd*u_k
    // => -Ad*x_k + I*x_{k+1} - Bd*u_k = 0
    for (int k = 0; k < N; ++k) {
        int row_idx = (k + 1) * nx;   // bắt đầu từ row 2
        int xk_idx  = k * nx;
        int xk1_idx = (k + 1) * nx;
        int uk_idx  = u_start + k * nu;

        // -Ad * x_k
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                if (std::abs(Ad(r, c)) > 1e-9) {
                    a_triplets.emplace_back(row_idx + r, xk_idx + c, -Ad(r, c));
                }
            }
        }

        // +I * x_{k+1}
        for (int r = 0; r < nx; ++r) {
            a_triplets.emplace_back(row_idx + r, xk1_idx + r, 1.0);
        }

        // -Bd * u_k
        for (int r = 0; r < nx; ++r) {
            if (std::abs(Bd(r)) > 1e-9) {
                a_triplets.emplace_back(row_idx + r, uk_idx, -Bd(r));
            }
        }

        // equality: 0
        l(row_idx + 0) = 0.0;
        u(row_idx + 0) = 0.0;
        l(row_idx + 1) = 0.0;
        u(row_idx + 1) = 0.0;
    }

    // 4c. Ràng buộc input: -max_steer_ <= delta_k <= max_steer_
    int ineq_start = n_eq;
    for (int k = 0; k < N; ++k) {
        int row = ineq_start + k;
        int col = u_start + k * nu;
        a_triplets.emplace_back(row, col, 1.0);
        l(row) = -max_steer_;
        u(row) =  max_steer_;
    }

    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());

    // --- 5. Setup OSQP & Solve (như cũ) ---
    if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }

    if (P_x_) free(P_x_); 
    if (P_i_) free(P_i_); 
    if (P_p_) free(P_p_);
    if (A_x_) free(A_x_); 
    if (A_i_) free(A_i_); 
    if (A_p_) free(A_p_);
    if (q_data_) free(q_data_); 
    if (l_data_) free(l_data_); 
    if (u_data_) free(u_data_);

    OSQPCscMatrix P_mat, A_mat;
    eigenToOSQPCsc(P,     P_mat, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_cons, A_mat, A_x_, A_i_, A_p_);

    q_data_ = (OSQPFloat*)malloc(sizeof(OSQPFloat) * n_vars);
    l_data_ = (OSQPFloat*)malloc(sizeof(OSQPFloat) * n_cons);
    u_data_ = (OSQPFloat*)malloc(sizeof(OSQPFloat) * n_cons);

    for (int i = 0; i < n_vars; ++i) q_data_[i] = (OSQPFloat)q(i);
    for (int i = 0; i < n_cons; ++i) {
        l_data_[i] = (OSQPFloat)l(i);
        u_data_[i] = (OSQPFloat)u(i);
    }

    if (settings_) free(settings_);
    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->alpha   = 1.0;

    OSQPInt exitflag = osqp_setup(
        &solver_, &P_mat, q_data_, &A_mat, l_data_, u_data_,
        (OSQPInt)n_cons, (OSQPInt)n_vars, settings_
    );

    if (exitflag != 0) {
        RCLCPP_ERROR(get_logger(), "OSQP Setup failed: %lld", (long long)exitflag);
        return {0.0};
    }

    osqp_solve(solver_);

    double delta_opt = 0.0;
    if (solver_->info->status_val == OSQP_SOLVED ||
        solver_->info->status_val == OSQP_SOLVED_INACCURATE) {
        delta_opt = (double)solver_->solution->x[u_start];
    } else {
        RCLCPP_WARN(get_logger(), "OSQP Failed. Status: %s", solver_->info->status);
    }

    return {delta_opt};
}

void MpcController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!has_path_ || path_points_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for /mpc_path...");
        return;
    }

    double x   = msg->pose.pose.position.x;
    double y   = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    // 1. Reference trên path
    size_t idx = findClosestPoint(x, y);
    double rx, ry, psi_ref;
    computeRefAtIndex(idx, rx, ry, psi_ref);
    
    // 2. Lỗi theo path
    double ey0, epsi0;
    computeErrorState(x, y, yaw, rx, ry, psi_ref, ey0, epsi0);

    // 3. Giải MPC cho (ey, epsi)
    Control u_opt = solveMPC(ey0, epsi0);
    double delta  = u_opt[0];

    // 4. Convert sang yaw_rate
    double omega = (desired_speed_ / wheel_base_) * std::tan(delta);

    // 5. Publish Twist như hệ thống cũ
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x  = desired_speed_;
    cmd.twist.angular.z = omega;


    // 6. Them logic dừng xe khi tới đích
    const auto &goal = path_points_.back();
    double dx_g = x - goal.first;
    double dy_g = y - goal.second;
    double dist_to_goal = std::sqrt(dx_g*dx_g + dy_g*dy_g);
    // Nếu đã vào vùng gần điểm cuối -> ép xe dừng
    if (dist_to_goal < goal_tolerance_ && idx >= path_points_.size() - 2) {
        cmd.twist.linear.x  = 0.0;
        cmd.twist.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "GOAL REACHED, STOP !!!!!!!1. dist=%.3f", dist_to_goal);
    }


    //7. Publish command
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),
    "LOG_COMPARE | x=%.3f y=%.3f yaw=%.3f | ref_x=%.3f ref_y=%.3f ref_yaw=%.3f | ey=%.3f epsi=%.3f",
    x, y, yaw,
    rx, ry, psi_ref,
    ey0, epsi0);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
