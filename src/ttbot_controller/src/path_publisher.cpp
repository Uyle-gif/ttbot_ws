#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // Tham số tên file CSV và frame
        this->declare_parameter<std::string>("path_file", "path_u_to_S.csv");
        this->declare_parameter<std::string>("frame_id", "map");

        path_file_ = this->get_parameter("path_file").as_string();
        frame_id_  = this->get_parameter("frame_id").as_string();

        // Publisher /mpc_path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);

        // Load file CSV
        loadCSV();

        // Timer publish định kỳ 1 Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::publishPath, this));
    }

private:
    std::string path_file_;
    std::string frame_id_;

    std::vector<std::pair<double,double>> points_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loadCSV()
    {
        std::string pkg_share;
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("ttbot_controller");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not get package share dir: %s", e.what());
            return;
        }

        std::string full_path = pkg_share + "/path/" + path_file_;
        RCLCPP_INFO(this->get_logger(), "Reading CSV: %s", full_path.c_str());

        std::ifstream file(full_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file %s", full_path.c_str());
            return;
        }

        points_.clear();
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            double x, y;
            char comma;
            if (ss >> x >> comma >> y) {
                points_.emplace_back(x, y);
            }
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), "Loaded %zu points", points_.size());
    }

    void publishPath()
    {
        if (points_.empty()) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = frame_id_;

        for (const auto &pt : points_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.first;
            pose.pose.position.y = pt.second;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
