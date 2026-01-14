// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include <cmath>
// #include <vector>

// class PathPublisher : public rclcpp::Node
// {
// public:
//     PathPublisher() : Node("path_publisher_node")
//     {
//         // 1. Cấu hình QoS: BẮT BUỘC phải có transient_local()
//         // Để khi publish xong và dừng lại, RViz mở sau vẫn nhận được dữ liệu cũ.
//         rclcpp::QoS qos_profile(1);
//         qos_profile.transient_local();
//         qos_profile.reliable();

//         path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

//         // Timer kích hoạt sau 1 giây
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             std::bind(&PathPublisher::timerCallback, this));

//         RCLCPP_INFO(this->get_logger(), "--> Node Started. Waiting to publish SQUARE once...");
//     }

// private:
//     void timerCallback()
//     {
//         // GỌI HÀM TẠO HÌNH VUÔNG
//         nav_msgs::msg::Path path_msg = generateSquarePath();
        
//         // Publish dữ liệu
//         path_pub_->publish(path_msg);
//         RCLCPP_INFO(this->get_logger(), "--> Published Square Path successfully.");

//         // 2. QUAN TRỌNG: Hủy timer ngay lập tức để không chạy lại lần 2
//         timer_->cancel();
//         RCLCPP_INFO(this->get_logger(), "--> Timer cancelled. Stopping updates.");
//     }

//     // Hàm phụ để thêm điểm vào Path cho gọn code
//     void addPointToPath(nav_msgs::msg::Path &path, double x, double y)
//     {
//         geometry_msgs::msg::PoseStamped pose;
//         pose.header = path.header;
//         pose.pose.position.x = x;
//         pose.pose.position.y = y;
//         pose.pose.position.z = 0.0;
        
//         pose.pose.orientation.w = 1.0;
//         pose.pose.orientation.x = 0.0;
//         pose.pose.orientation.y = 0.0;
//         pose.pose.orientation.z = 0.0;

//         path.poses.push_back(pose);
//     }

//     nav_msgs::msg::Path generateSquarePath()
//     {
//         nav_msgs::msg::Path path;
//         path.header.stamp = this->now();
//         path.header.frame_id = "odom"; 

//         double side_len = 10.0; 
//         double step = 0.1;      

//         // --- CẠNH 1: (0,0) -> (10,0) ---
//         for (double x = 0; x <= side_len; x += step) {
//             addPointToPath(path, x, 0.0);
//         }

//         // --- CẠNH 2: (10,0) -> (10,10) ---
//         for (double y = 0; y <= side_len; y += step) {
//             addPointToPath(path, side_len, y);
//         }

//         // --- CẠNH 3: (10,10) -> (0,10) ---
//         for (double x = side_len; x >= 0; x -= step) {
//             addPointToPath(path, x, side_len);
//         }

//         // --- CẠNH 4: (0,10) -> (0,0) ---
//         for (double y = side_len; y >= 0; y -= step) {
//             addPointToPath(path, 0.0, y);
//         }

//         return path;
//     }

//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PathPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <vector>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher_node")
    {
        // QoS transient_local: Lưu tin nhắn cho người đến sau (RViz)
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        qos_profile.reliable();

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

        // Timer chạy sau 1s
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "--> Node Started. Waiting to publish CIRCLE Path...");
    }

private:
    void timerCallback()
    {
        // 1. Tạo path hình TRÒN
        nav_msgs::msg::Path path_msg = generateCirclePath();
        
        // 2. Publish
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "--> Published Circle Path (R=6m).");

        // 3. Dừng timer (chỉ pub 1 lần)
        timer_->cancel();
    }

    void addPointToPath(nav_msgs::msg::Path &path, double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        // Tính quaternion từ yaw để mũi tên trên RViz chỉ đúng hướng đi
        // (Không bắt buộc nếu chỉ cần vị trí, nhưng tốt cho debug)
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);

        path.poses.push_back(pose);
    }

    nav_msgs::msg::Path generateCirclePath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom"; // Hoặc "map"

        // --- CẤU HÌNH HÌNH TRÒN ---
        double radius = 10.0;        // Bán kính 15m
        double center_x = 0.0;      // Tâm X
        double center_y = 0.0;      // Tâm Y
        double step = 0.05;         // Độ phân giải góc (radian)

        // Vòng lặp từ 0 đến 2*PI (một vòng tròn)
        for (double theta = 0.0; theta <= 2 * M_PI; theta += step) {
            double x = center_x + radius * std::cos(theta);
            double y = center_y + radius * std::sin(theta);
            
            // Tính hướng tiếp tuyến (yaw) tại điểm đó
            // Hướng tiếp tuyến của đường tròn là theta + PI/2
            double yaw = theta + M_PI_2; 

            addPointToPath(path, x, y, yaw);
        }

        return path;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}