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
        // 1. QoS Transient Local: Để RViz vào sau vẫn thấy đường
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        qos_profile.reliable();

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

        // Timer kích hoạt sau 1 giây
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "--> Node Started. Waiting to publish ROUNDED SQUARE (No Yaw)...");
    }

private:
    void timerCallback()
    {
        // GỌI HÀM TẠO HÌNH VUÔNG BO TRÒN
        nav_msgs::msg::Path path_msg = generateRoundedSquarePath();
        
        // Publish dữ liệu
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "--> Published Rounded Square Path (R=2m).");

        // Hủy timer ngay lập tức để chỉ publish 1 lần
        timer_->cancel();
    }

    // Hàm thêm điểm đơn giản (Chỉ X, Y)
    void addPointToPath(nav_msgs::msg::Path &path, double x, double y)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        // Orientation mặc định (w=1.0) không tính toán xoay
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        path.poses.push_back(pose);
    }

    nav_msgs::msg::Path generateRoundedSquarePath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom"; 

        double side = 10.0;     // Độ dài cạnh hình vuông (m)
        double R = 2.0;         // Bán kính bo góc (R=2m cho mượt)
        double step = 0.05;     // Độ mịn của đường (m)

        // Nguyên tắc: Đi thẳng -> Cua tròn -> Đi thẳng -> ...
        // Tọa độ 4 đỉnh góc bo tròn:
        // Góc 1 (Phải dưới): Tâm (side-R, R)
        // Góc 2 (Phải trên): Tâm (side-R, side-R)
        // Góc 3 (Trái trên): Tâm (R, side-R)
        // Góc 4 (Trái dưới): Tâm (R, R)

        // 1. Cạnh Đáy: Từ x=R -> x=side-R
        for (double x = R; x <= side - R; x += step) {
            addPointToPath(path, x, 0.0);
        }

        // 2. Góc Phải-Dưới: Cung tròn từ -90 -> 0 độ
        for (double t = -M_PI_2; t <= 0; t += step) {
            double cx = side - R;
            double cy = R;
            addPointToPath(path, cx + R * std::cos(t), cy + R * std::sin(t));
        }

        // 3. Cạnh Phải: Từ y=R -> y=side-R
        for (double y = R; y <= side - R; y += step) {
            addPointToPath(path, side, y);
        }

        // 4. Góc Phải-Trên: Cung tròn từ 0 -> 90 độ
        for (double t = 0; t <= M_PI_2; t += step) {
            double cx = side - R;
            double cy = side - R;
            addPointToPath(path, cx + R * std::cos(t), cy + R * std::sin(t));
        }

        // 5. Cạnh Trên: Từ x=side-R -> x=R
        for (double x = side - R; x >= R; x -= step) {
            addPointToPath(path, x, side);
        }

        // 6. Góc Trái-Trên: Cung tròn từ 90 -> 180 độ
        for (double t = M_PI_2; t <= M_PI; t += step) {
            double cx = R;
            double cy = side - R;
            addPointToPath(path, cx + R * std::cos(t), cy + R * std::sin(t));
        }

        // 7. Cạnh Trái: Từ y=side-R -> y=R
        for (double y = side - R; y >= R; y -= step) {
            addPointToPath(path, 0.0, y);
        }

        // 8. Góc Trái-Dưới: Cung tròn từ 180 -> 270 độ (về lại điểm nối đáy)
        for (double t = M_PI; t <= 3*M_PI_2; t += step) {
            double cx = R;
            double cy = R;
            addPointToPath(path, cx + R * std::cos(t), cy + R * std::sin(t));
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
//         // QoS transient_local: Lưu tin nhắn cho người đến sau (RViz)
//         rclcpp::QoS qos_profile(1);
//         qos_profile.transient_local();
//         qos_profile.reliable();

//         path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

//         // Timer chạy sau 1s
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             std::bind(&PathPublisher::timerCallback, this));

//         RCLCPP_INFO(this->get_logger(), "--> Node Started. Waiting to publish CIRCLE Path...");
//     }

// private:
//     void timerCallback()
//     {
//         // 1. Tạo path hình TRÒN
//         nav_msgs::msg::Path path_msg = generateCirclePath();
        
//         // 2. Publish
//         path_pub_->publish(path_msg);
//         RCLCPP_INFO(this->get_logger(), "--> Published Circle Path (R=6m).");

//         // 3. Dừng timer (chỉ pub 1 lần)
//         timer_->cancel();
//     }

//     void addPointToPath(nav_msgs::msg::Path &path, double x, double y, double yaw)
//     {
//         geometry_msgs::msg::PoseStamped pose;
//         pose.header = path.header;
//         pose.pose.position.x = x;
//         pose.pose.position.y = y;
//         pose.pose.position.z = 0.0;
        
//         // Tính quaternion từ yaw để mũi tên trên RViz chỉ đúng hướng đi
//         // (Không bắt buộc nếu chỉ cần vị trí, nhưng tốt cho debug)
//         pose.pose.orientation.z = std::sin(yaw / 2.0);
//         pose.pose.orientation.w = std::cos(yaw / 2.0);

//         path.poses.push_back(pose);
//     }

//     nav_msgs::msg::Path generateCirclePath()
//     {
//         nav_msgs::msg::Path path;
//         path.header.stamp = this->now();
//         path.header.frame_id = "odom"; // Hoặc "map"

//         // --- CẤU HÌNH HÌNH TRÒN ---
//         double radius = 10.0;        // Bán kính 15m
//         double center_x = 0.0;      // Tâm X
//         double center_y = 0.0;      // Tâm Y
//         double step = 0.05;         // Độ phân giải góc (radian)

//         // Vòng lặp từ 0 đến 2*PI (một vòng tròn)
//         for (double theta = 0.0; theta <= 2 * M_PI; theta += step) {
//             double x = center_x + radius * std::cos(theta);
//             double y = center_y + radius * std::sin(theta);
            
//             // Tính hướng tiếp tuyến (yaw) tại điểm đó
//             // Hướng tiếp tuyến của đường tròn là theta + PI/2
//             double yaw = theta + M_PI_2; 

//             addPointToPath(path, x, y, yaw);
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