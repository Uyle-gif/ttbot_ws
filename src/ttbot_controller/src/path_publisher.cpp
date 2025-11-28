#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // 1. Khai báo tham số
        this->declare_parameter("path_file", "path_u_to_S.csv"); // Tên file trong thư mục 'path'
        this->declare_parameter("frame_id", "map"); // Frame tham chiếu (map hoặc odom)
        this->declare_parameter("publish_rate", 1.0); // Hz (Path tĩnh không cần gửi nhanh)

        // 2. Publisher topic /mpc_path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);

        // 3. Load Path
        loadPath();

        // 4. Timer để publish định kỳ (giúp RViz hoặc Controller mới bật lên vẫn nhận được)
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&PathPublisher::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Path Publisher Initialized. Topic: /mpc_path");
    }

private:
    void loadPath()
    {
        std::string filename = this->get_parameter("path_file").as_string();
        std::string frame_id = this->get_parameter("frame_id").as_string();

        path_msg_.header.frame_id = frame_id;

        // --- Tìm đường dẫn tuyệt đối tới file CSV trong share directory ---
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("ttbot_controller");
        std::string full_path = package_share_directory + "/path/" + filename;

        std::ifstream file(full_path);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                if (line.empty()) continue;
                std::stringstream ss(line);
                std::string value;
                std::vector<double> row;

                while (std::getline(ss, value, ',')) {
                    try {
                        row.push_back(std::stod(value));
                    } catch (...) { continue; }
                }

                if (row.size() >= 2) {
                    addPointToPath(row[0], row[1]);
                }
            }
            file.close();
            RCLCPP_INFO(this->get_logger(), "Loaded custom path from: %s (%zu points)", full_path.c_str(), path_msg_.poses.size());
        } 
        else {
            // --- Fallback: Tạo đường số 8 nếu không có file ---
            RCLCPP_WARN(this->get_logger(), "File not found: %s. Generating Figure-8 path.", full_path.c_str());
            generateFigure8Path();
        }
    }

    void generateFigure8Path()
    {
        // Tạo 100 điểm hình số 8
        for (int i = 0; i < 200; ++i) {
            double t = 2.0 * M_PI * i / 200.0;
            double x = 4.0 * std::sin(t);       // Rộng 4m
            double y = 2.0 * std::sin(2.0 * t); // Cao 2m
            addPointToPath(x, y);
        }
    }

    void addPointToPath(double x, double y)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path_msg_.header.frame_id;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        // Orientation mặc định (có thể tính toán nếu cần, nhưng Controller tự tính heading rồi)
        pose.pose.orientation.w = 1.0; 

        path_msg_.poses.push_back(pose);
    }

    void timerCallback()
    {
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}