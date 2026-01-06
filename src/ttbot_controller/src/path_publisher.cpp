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
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        qos_profile.reliable();

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "--> Path Publisher: FIGURE-8 (Infinity Shape)");
    }

private:
    void timerCallback()
    {
        // GỌI HÀM TẠO SỐ 8
        nav_msgs::msg::Path path_msg = generateFigure8Path();
        path_pub_->publish(path_msg);
    }

    nav_msgs::msg::Path generateFigure8Path()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom"; 

        int total_points = 1200; // Tăng số điểm lên để đường cong mượt hơn ở chỗ cua gắt
        double scale = 5.0;      // Kích thước của số 8 (Rộng ~10m)

        for (int i = 0; i < total_points; ++i) {
            double t = 2.0 * M_PI * i / (double)total_points;

            // --- CÔNG THỨC HÌNH SỐ 8 (Lemniscate of Gerono dạng đơn giản) ---
            // x chạy qua lại như con lắc (cos)
            // y chạy nhanh gấp đôi (sin 2t) để tạo nút thắt
            double x = scale * std::cos(t);
            double y = scale * std::sin(2.0 * t) / 2.0; 

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            // Quaternion mặc định
            pose.pose.orientation.w = 1.0;

            path.poses.push_back(pose);
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