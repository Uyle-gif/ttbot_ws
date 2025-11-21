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
    // Tái sử dụng normalizeAngle từ Stanley (hoặc định nghĩa lại)
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
}


// Khởi tạo MPC Controller
MpcController::MpcController()
: Node("mpc_controller")
{
    // 1. Khai báo và Load Parameters (Dùng lại của Stanley và thêm MPC)
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("wheel_base", 0.8);
    this->declare_parameter("max_steer_deg", 60.0);
    this->declare_parameter("path_file", "path.csv");

    // MPC Parameters
    this->declare_parameter("N_p", 20);      // Prediction Horizon
    this->declare_parameter("dt_mpc", 0.1);  // Bước thời gian (s)
    this->declare_parameter("Q_x", 10.0);    // Trọng số theo dõi x/y
    this->declare_parameter("Q_psi", 50.0);  // Trọng số theo dõi góc yaw
    this->declare_parameter("R_omega", 1.0); // Trọng số điều khiển omega

    // Load
    desired_speed_ = this->get_parameter("desired_speed").as_double();
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
    path_file_     = this->get_parameter("path_file").as_string();

    N_p_ = this->get_parameter("N_p").as_int();
    dt_mpc_ = this->get_parameter("dt_mpc").as_double();
    Q_x_ = this->get_parameter("Q_x").as_double();
    Q_psi_ = this->get_parameter("Q_psi").as_double();
    R_omega_ = this->get_parameter("R_omega").as_double();

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
    loadPathFromCSV(); 

    // 3. Subscribers & Publishers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&MpcController::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_controller/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "MPC Controller initialized with N_p=%d, dt=%.2f", N_p_, dt_mpc_);
}

// =================================================================================
// Destructor: Dọn dẹp bộ nhớ thủ công (Bắt buộc với OSQP)
// =================================================================================
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
  size_t closest_index = 0;
  double min_dist_sq   = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < path_points_.size(); ++i) {
    double dx = x - path_points_[i].first;
    double dy = y - path_points_[i].second;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_index = i;
    }
  }
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

    OSQPInt nnz = mat.nonZeros();
    OSQPInt cols = mat.cols();
    OSQPInt rows = mat.rows();

    // Thay c_malloc bằng malloc
    out_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * nnz);
    out_i = (OSQPInt*)malloc(sizeof(OSQPInt) * nnz);
    out_p = (OSQPInt*)malloc(sizeof(OSQPInt) * (cols + 1));

    // 2. Copy dữ liệu
    for (int k = 0; k < nnz; k++) {
        out_x[k] = (OSQPFloat)mat.valuePtr()[k];
        out_i[k] = (OSQPInt)mat.innerIndexPtr()[k];
    }
    for (int k = 0; k < cols + 1; k++) {
        out_p[k] = (OSQPInt)mat.outerIndexPtr()[k];
    }

    // 3. Gán vào struct tạm
    out_mat.m = rows;
    out_mat.n = cols;
    out_mat.p = out_p;
    out_mat.i = out_i;
    out_mat.x = out_x;
    out_mat.nzmax = nnz;
    out_mat.nz = -1; // CSC format
    out_mat.owned = 0; // User owned memory
}
// =================================================================================
// SOLVE MPC (CORE LOGIC)
// =================================================================================
Control MpcController::solveMPC(const State& current_state)
{
    // --- 1. Cấu hình kích thước ---
    int nx = 3; // x, y, psi
    int nu = 1; // omega
    int N = N_p_;
    
    // Số biến: [x0, ..., xN] (N+1 state) + [u0, ..., u(N-1)] (N input)
    // Tổng cộng: z = [x0, x1, ... xN, u0, u1, ... u(N-1)]
    // Note: Cách sắp xếp biến: State trước, Input sau.
    int n_vars = (N + 1) * nx + N * nu;
    
    // Số ràng buộc:
    // - Động lực học: N bước * nx trạng thái = N * nx (Phương trình đẳng thức)
    // - Trạng thái đầu: nx (x0 = current)
    // - Giới hạn Input: N * nu
    int n_eq = (N + 1) * nx; // Bao gồm cả init state
    int n_ineq = N * nu;
    int n_cons = n_eq + n_ineq;

    // --- 2. Tuyến tính hóa & Reference ---
    linearizeModel(desired_speed_, current_state[2], dt_mpc_);
    
    // Map lại ma trận Ad, Bd từ vector phẳng
    Eigen::Map<Eigen::Matrix<double, 3, 3>> Ad(A_flat_.data());
    Eigen::Map<Eigen::Matrix<double, 3, 1>> Bd(B_flat_.data());

    size_t start_idx = findClosestPoint(current_state[0], current_state[1]);

    // --- 3. Xây dựng Ma trận P (Cost) & Vector q ---
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

    // Phần State (Q matrix)
    for (int k = 0; k <= N; ++k) {
        int offset = k * nx;
        p_triplets.emplace_back(offset + 0, offset + 0, Q_x_);
        p_triplets.emplace_back(offset + 1, offset + 1, Q_x_);
        p_triplets.emplace_back(offset + 2, offset + 2, Q_psi_);

        // Tính q = -Q * x_ref
        size_t path_idx = std::min(start_idx + k, path_points_.size() - 1);
        double rx = path_points_[path_idx].first;
        double ry = path_points_[path_idx].second;
        // Yaw reference (đơn giản hóa bằng 0 hoặc tính đạo hàm đường đi)
        double rpsi = 0.0; 
        if (path_idx < path_points_.size() - 1) {
            rpsi = std::atan2(path_points_[path_idx+1].second - ry, 
                              path_points_[path_idx+1].first - rx);
        }
        // Chuẩn hóa góc lệch
        double delta_psi = normalizeAngle(rpsi);

        q(offset + 0) = -Q_x_ * rx;
        q(offset + 1) = -Q_x_ * ry;
        q(offset + 2) = -Q_psi_ * delta_psi;
    }

    // Phần Input (R matrix)
    int u_start = (N + 1) * nx;
    for (int k = 0; k < N; ++k) {
        int offset = u_start + k * nu;
        p_triplets.emplace_back(offset, offset, R_omega_);
    }
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // --- 4. Xây dựng Ma trận A (Constraints) & Vectors l, u ---
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_cons);

    // 4a. Ràng buộc trạng thái đầu (x0 = current) -> Rows: 0, 1, 2
    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
        l(i) = current_state[i];
        u(i) = current_state[i];
    }

    // 4b. Ràng buộc động lực học: x_{k+1} = Ad*x_k + Bd*u_k
    // => -Ad*x_k + I*x_{k+1} - Bd*u_k = 0
    for (int k = 0; k < N; ++k) {
        int row_idx = (k + 1) * nx; // Bắt đầu từ row 3 (x1)
        int xk_idx = k * nx;
        int xk1_idx = (k + 1) * nx;
        int uk_idx = u_start + k * nu;

        // -Ad * x_k
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                if (std::abs(Ad(r, c)) > 1e-5)
                    a_triplets.emplace_back(row_idx + r, xk_idx + c, -Ad(r, c));
            }
        }
        // +I * x_{k+1}
        for (int r = 0; r < nx; ++r) {
            a_triplets.emplace_back(row_idx + r, xk1_idx + r, 1.0);
        }
        // -Bd * u_k
        for (int r = 0; r < nx; ++r) {
            if (std::abs(Bd(r)) > 1e-5)
                a_triplets.emplace_back(row_idx + r, uk_idx, -Bd(r));
        }

        // Bounds = 0 (Equality)
        for (int r = 0; r < nx; ++r) {
            l(row_idx + r) = 0.0;
            u(row_idx + r) = 0.0;
        }
    }

    // 4c. Ràng buộc Input (Inequality): -max <= u <= max
    int ineq_start = n_eq;
    for (int k = 0; k < N; ++k) {
        int row = ineq_start + k;
        int col = u_start + k;
        a_triplets.emplace_back(row, col, 1.0);
        l(row) = -max_omega_;
        u(row) = max_omega_;
    }
    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());

    // --- 5. Setup OSQP & Solve ---
    
    // Clean old memory (Thay c_free bằng free và xuống dòng)
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

    // Prepare structs
    OSQPCscMatrix P_mat, A_mat;
    eigenToOSQPCsc(P, P_mat, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_cons, A_mat, A_x_, A_i_, A_p_);

    // Copy vectors
    q_data_ = (OSQPFloat*)malloc(sizeof(OSQPFloat) * n_vars);
    l_data_ = (OSQPFloat*)malloc(sizeof(OSQPFloat) * n_cons);
    u_data_ = (OSQPFloat*)malloc(sizeof(OSQPFloat) * n_cons);

    for(int i=0; i<n_vars; ++i) q_data_[i] = (OSQPFloat)q(i);
    for(int i=0; i<n_cons; ++i) {
        l_data_[i] = (OSQPFloat)l(i);
        u_data_[i] = (OSQPFloat)u(i);
    }

    // Settings
    if (settings_) free(settings_);
    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings)); // Sửa ở đây
    osqp_set_default_settings(settings_);
    settings_->verbose = 0; 
    settings_->alpha = 1.0;

    // OSQP SETUP (v1.0 API)
    OSQPInt exitflag = osqp_setup(
        &solver_, &P_mat, q_data_, &A_mat, l_data_, u_data_, n_cons, n_vars, settings_
    );

    if (exitflag != 0) {
        RCLCPP_ERROR(get_logger(), "OSQP Setup failed: %lld", (long long)exitflag);
        return {0.0};
    }

    // SOLVE
    osqp_solve(solver_);

    // --- 6. Lấy kết quả ---
    double optimal_omega = 0.0;
    if (solver_->info->status_val == OSQP_SOLVED || 
        solver_->info->status_val == OSQP_SOLVED_INACCURATE) 
    {
        // Biến điều khiển u0 nằm ở vị trí u_start
        optimal_omega = (double)solver_->solution->x[u_start];
    } else {
        RCLCPP_WARN(get_logger(), "OSQP Failed. Status: %s", solver_->info->status);
    }

    return {optimal_omega};
}

void MpcController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (path_points_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path loaded yet!");
        return;
    }

    // Lấy trạng thái hiện tại
    double x   = msg->pose.pose.position.x;
    double y   = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    
    State current_state = {x, y, yaw};

    // 1. Giải MPC
    Control optimal_control = solveMPC(current_state);
    double optimal_omega = optimal_control[0];

    // 2. Xuất bản lệnh
    geometry_msgs::msg::TwistStamped cmd_msg;
    cmd_msg.header.stamp    = this->now();
    cmd_msg.header.frame_id = "base_link";
    cmd_msg.twist.linear.x  = desired_speed_; // Vận tốc tuyến tính cố định
    cmd_msg.twist.angular.z = optimal_omega;  // Vận tốc góc từ MPC
    
    cmd_pub_->publish(cmd_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "MPC: omega=%.3f rad/s, x=%.2f, y=%.2f",
        optimal_omega, x, y);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}