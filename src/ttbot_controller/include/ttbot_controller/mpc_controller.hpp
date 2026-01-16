#ifndef TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <utility>
#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <osqp/osqp.h>

using Control = std::vector<double>; 

class MpcController : public rclcpp::Node
{
public:
    MpcController();
    ~MpcController();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);    

    Control solveMPC(double ey0, double epsi0, double v_ref);
    
    void linearizeErrorModel(double v, double dt);
    
    size_t findClosestPoint(double x, double y);
    void computeReference(size_t idx, double &rx, double &ry, double &rpsi);
    void computeErrorState(double x, double y, double yaw,
                           double rx, double ry, double rpsi,
                           double &ey, double &epsi);

    void freeOSQPMemory();
    void eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                        OSQPCscMatrix& out_mat,
                        OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p);

    double desired_speed_;
    double wheel_base_;
    double max_steer_;     
    double max_omega_;     

    int    N_p_;      
    double dt_mpc_;   
    double Q_ey_;
    double Q_epsi_;
    double R_delta_;

    double goal_tolerance_;
    bool reached_goal_;

    std::vector<std::pair<double, double>> path_points_;
    size_t current_index_;
    bool has_path_;

    Eigen::Matrix2d Ad_;
    Eigen::Vector2d Bd_;

    OSQPSolver* solver_   = nullptr;
    OSQPSettings* settings_ = nullptr;
    
    OSQPFloat* P_x_ = nullptr; OSQPInt* P_i_ = nullptr; OSQPInt* P_p_ = nullptr;
    OSQPFloat* A_x_ = nullptr; OSQPInt* A_i_ = nullptr; OSQPInt* A_p_ = nullptr;
    OSQPFloat* q_data_ = nullptr;
    OSQPFloat* l_data_ = nullptr;
    OSQPFloat* u_data_ = nullptr;

    // ==== ROS ====
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_tuning_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_cte_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_heading_pub_;
    double current_pose_x_ = 0.0;
    double current_pose_y_ = 0.0;
    double current_pose_yaw_ = 0.0;
    bool has_odom_ = false;

    double mpc_ref_x_ = 0.0;
    double mpc_ref_y_ = 0.0;
    double mpc_ref_yaw_ = 0.0;
    bool is_ref_set_ = false; 
};

#endif // TTBOT_CONTROLLER_MPC_CONTROLLER_HPP_