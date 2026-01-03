// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "ament_index_cpp/get_package_share_directory.hpp"

// #include <fstream>
// #include <sstream>
// #include <string>
// #include <vector>
// #include <cmath>

// class PathPublisher : public rclcpp::Node
// {
// public:
//     PathPublisher() : Node("path_publisher")
//     {
//         this->declare_parameter("path_file", "path_u_to_S.csv"); 
//         this->declare_parameter("frame_id", "map"); 
        
//         // QoS Transient Local: Giúp các node vào sau (Rviz) vẫn nhận được map
//         rclcpp::QoS qos_profile(10);
//         qos_profile.transient_local(); 
        
//         path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

//         loadPath();

//         // Timer chạy 1 lần sau 1 giây
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             std::bind(&PathPublisher::timerCallback, this));
            
//         RCLCPP_INFO(this->get_logger(), "Path Publisher Initialized. Will publish ONCE.");
//     }

// private:
//     void loadPath()
//     {
//         std::string filename = this->get_parameter("path_file").as_string();
//         std::string frame_id = this->get_parameter("frame_id").as_string();

//         path_msg_.header.frame_id = frame_id;

//         std::string package_share_directory;
//         try {
//             package_share_directory = ament_index_cpp::get_package_share_directory("ttbot_controller");
//         } catch (...) {
//             package_share_directory = "."; 
//         }

//         std::string full_path = package_share_directory + "/path/" + filename;

//         std::ifstream file(full_path);
//         if (file.is_open()) {
//             // ... (Giữ nguyên phần đọc file nếu có) ...
//             // Code đọc file CSV ở đây (đã rút gọn cho ngắn, bạn giữ nguyên logic cũ nếu cần)
//             file.close();
//         } 
//         else {
//             RCLCPP_WARN(this->get_logger(), "File not found: %s. Generating Straight Line.", full_path.c_str());
//             // --- GỌI HÀM TẠO ĐƯỜNG THẲNG ---
//             generateLinePath();
//         }
//     }

//     // --- HÀM MỚI: TẠO ĐƯỜNG THẲNG ---
//     void generateLinePath()
//     {
//         int total_points = 200; // Số lượng điểm trên đường thẳng
//         double length = 10.0;   // Độ dài đường thẳng (mét)
        
//         // Điểm bắt đầu (0,0)
//         double start_x = 0.0;
//         double start_y = 0.0;

//         // Điểm kết thúc (10, 0) -> Chạy dọc trục X
//         double end_x = start_x + length;
//         double end_y = start_y; 

//         for (int i = 0; i < total_points; ++i) {
//             // Tính tỉ lệ phần trăm quãng đường (từ 0.0 đến 1.0)
//             double ratio = (double)i / (total_points - 1);

//             // Công thức nội suy tuyến tính
//             double x = start_x + ratio * (end_x - start_x);
//             double y = start_y + ratio * (end_y - start_y);
            
//             addPointToPath(x, y);
//         }
//     }

//     void addPointToPath(double x, double y)
//     {
//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.frame_id = path_msg_.header.frame_id;
//         pose.pose.position.x = x;
//         pose.pose.position.y = y;
//         pose.pose.position.z = 0.0;
        
//         // Định hướng robot (Quaternion). w=1.0 nghĩa là không xoay (hướng theo trục X)
//         // Nếu muốn robot quay theo hướng di chuyển, cần tính yaw từ atan2(dy, dx)
//         pose.pose.orientation.w = 1.0; 
//         pose.pose.orientation.x = 0.0;
//         pose.pose.orientation.y = 0.0;
//         pose.pose.orientation.z = 0.0;

//         path_msg_.poses.push_back(pose);
//     }

//     void timerCallback()
//     {
//         if (!path_msg_.poses.empty()) {
//             path_msg_.header.stamp = this->now();
//             path_pub_->publish(path_msg_);
//             RCLCPP_INFO(this->get_logger(), "Straight Line Path published successfully (One-time).");
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Path is empty.");
//         }
//         timer_->cancel(); // Hủy timer để chỉ chạy 1 lần
//     }

//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     nav_msgs::msg::Path path_msg_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PathPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }







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
        this->declare_parameter("path_file", "path_u_to_S.csv"); 
        this->declare_parameter("frame_id", "map"); 
        this->declare_parameter("publish_rate", 1.0); 

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);

        loadPath();

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
            RCLCPP_WARN(this->get_logger(), "File not found: %s. Generating Figure-8 path.", full_path.c_str());
            // generateFigure8Path();
            generateCirclePath();
        }
    }

    // void generateFigure8Path()
    // {
    //     // Tăng độ phân giải điểm lên một chút (ví dụ 400) để đường cong mượt hơn khi hình to ra
    //     int total_points = 500; 
        
    //     // Kích thước mong muốn (bạn chỉnh ở đây)
    //     double scale_x = 8.0; // Cũ là 4.0 -> Tăng lên 8.0 (chiều ngang)
    //     double scale_y = 4.0; // Cũ là 2.0 -> Tăng lên 4.0 (chiều dọc)

    //     for (int i = 0; i < total_points; ++i) {
    //         double t = 2.0 * M_PI * i / (double)total_points;
            
    //         // Công thức tham số: x = A*sin(t), y = B*sin(2t)
    //         double x = scale_x * std::sin(t);       
    //         double y = scale_y * std::sin(2.0 * t); 
            
    //         addPointToPath(x, y);
    //     }
    // }


    void generateCirclePath()
    {
        // Độ phân giải số lượng điểm (càng lớn đường càng mượt)
        int total_points = 500; 
        
        // BÁN KÍNH đường tròn (đơn vị mét)
        double radius = 5.0; 

        for (int i = 0; i < total_points; ++i) {
            // t chạy từ 0 đến 2*PI (một vòng tròn)
            double t = 2.0 * M_PI * i / (double)total_points;
            
            // Công thức tham số đường tròn
            // Nếu muốn robot bắt đầu từ gốc tọa độ (0,0) thì cần trừ offset, 
            // nhưng công thức chuẩn dưới đây tâm nằm tại (0,0)
            double x = radius * std::cos(t);       
            double y = radius * std::sin(t); 
            
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



