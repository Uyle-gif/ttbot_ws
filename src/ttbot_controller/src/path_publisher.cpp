#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("file_path", "");

        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string file_path = this->get_parameter("file_path").as_string();

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
        
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Missing 'file_path' parameter!");
            return;
        }

        if (load_csv(file_path)) {
            timer_ = this->create_wall_timer(
                1000ms, [this]() {
                    this->publish_path_once();
                    this->timer_->cancel(); 
                });
            
            RCLCPP_INFO(this->get_logger(), "Path Publisher Started. Waiting 1 second to publish path (One-shot)...");
        }
    }

private:
    bool load_csv(const std::string& file_path)
    {
        path_.header.frame_id = frame_id_;
        path_.poses.clear();

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return false;
        }

        std::string line;
        int point_count = 0;

        while (std::getline(file, line)) {
            if (line.empty() || isalpha(line[0]) || line[0] == '#') continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;

            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (...) {
                    continue; 
                }
            }

            if (values.size() >= 2) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = values[0];
                pose.pose.position.y = values[1];
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; 

                path_.poses.push_back(pose);
                point_count++;
            }
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), ">>> Successfully loaded %d coordinates from CSV file <<<", point_count);
        return true;
    }

    void publish_path_once()
    {
        path_.header.stamp = this->now();
        
        for (auto& pose : path_.poses) {
            pose.header.stamp = path_.header.stamp;
        }
        
        publisher_->publish(path_);
        RCLCPP_INFO(this->get_logger(), "--- PATH PUBLISHED SUCCESSFULLY ---");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    std::string frame_id_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}