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

        // Cập nhật Log báo hiệu là hình tròn
        RCLCPP_INFO(this->get_logger(), "--> Path Publisher: BIG CIRCLE (R=10m)");
    }

private:
    void timerCallback()
    {
        // GỌI HÀM TẠO HÌNH TRÒN
        nav_msgs::msg::Path path_msg = generateCirclePath();
        path_pub_->publish(path_msg);
    }

    nav_msgs::msg::Path generateCirclePath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom"; 

        int total_points = 1200; // Số lượng điểm
        double radius = 10.0;    // Bán kính to (10m -> Đường kính 20m)

        for (int i = 0; i < total_points; ++i) {
            double t = 2.0 * M_PI * i / (double)total_points;

            // --- CÔNG THỨC HÌNH TRÒN ---
            // x = R * cos(t)
            // y = R * sin(t)
            double x = radius * std::cos(t);
            double y = radius * std::sin(t); 

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            // Tính toán Orientation (Heading) cho từng điểm
            // Để robot biết hướng mũi tên tiếp tuyến với đường tròn (tuỳ chọn, nhưng tốt cho MPC)
            // Nếu chỉ cần path hiển thị thì w=1.0 là đủ.
            // Ở đây tôi giữ w=1.0 như code cũ của bạn.
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