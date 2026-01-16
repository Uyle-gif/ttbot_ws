#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        this->declare_parameter("frame_id", "map");
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Tạo Publisher với QoS mặc định (Reliable)
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
        
        // Tạo đường đi
        generate_rounded_u_path();

        // Timer sẽ kích hoạt sau 1 giây để đảm bảo hệ thống đã sẵn sàng kết nối
        // Sau đó nó sẽ publish 1 lần và tự tắt.
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&PathPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Path Publisher Started. Waiting 1s to publish ONCE...");
    }

private:
    void generate_rounded_u_path()
    {
        path_.header.frame_id = frame_id_;
        path_.poses.clear();
        double step = 0.1; // Mật độ điểm 10cm

        // 1. Đi thẳng 15m
        for (double x = 0.0; x <= 15.0; x += step) add_point(x, 0.0);

        // 2. Cua trái bo tròn (R=5m)
        double R = 5.0, cx1 = 15.0, cy1 = 5.0;
        for (double a = -M_PI_2; a <= 0.0; a += 0.05) 
            add_point(cx1 + R * cos(a), cy1 + R * sin(a));

        // 3. Đi thẳng lên 5m
        for (double y = 5.0; y <= 10.0; y += step) add_point(20.0, y);

        // 4. Cua trái quay đầu (R=5m)
        double cx2 = 15.0, cy2 = 10.0;
        for (double a = 0.0; a <= M_PI_2; a += 0.05) 
            add_point(cx2 + R * cos(a), cy2 + R * sin(a));

        // 5. Đi thẳng về đích (X = -5m)
        for (double x = 15.0; x >= -5.0; x -= step) add_point(x, 15.0);
    }

    void add_point(double x, double y) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0; 
        path_.poses.push_back(pose);
    }

    void timer_callback()
    {
        // Kiểm tra an toàn: Nếu đã gửi rồi thì thôi (dù timer đáng lẽ đã bị hủy)
        if (is_published_) return;

        // Cập nhật thời gian thực
        path_.header.stamp = this->now();
        
        // PUBLISH
        publisher_->publish(path_);
        
        RCLCPP_INFO(this->get_logger(), ">>> PATH PUBLISHED SUCCESSFULLY (1 TIME). STOPPING TIMER. <<<");
        
        // Đánh dấu đã gửi và HỦY TIMER NGAY LẬP TỨC
        is_published_ = true;
        timer_->cancel();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    std::string frame_id_;
    nav_msgs::msg::Path path_;
    bool is_published_ = false; // Cờ đánh dấu
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}