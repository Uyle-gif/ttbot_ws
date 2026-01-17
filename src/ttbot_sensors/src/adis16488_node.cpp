#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class Adis16488Node : public rclcpp::Node
{
public:
  Adis16488Node()
  : Node("adis16488_driver"), serial_fd_(-1)
  {
    // 1. Khai báo tham số
    this->declare_parameter("port", "/dev/ttbot_imu");
    this->declare_parameter("baudrate", 460800);
    this->declare_parameter("frame_id", "imu_link");
    
    // --- THÊM: Tham số giới hạn tần số ---
    this->declare_parameter("publish_rate", 50.0); // Mặc định 50Hz (Đủ cho EKF)

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    
    // Tính toán khoảng thời gian tối thiểu giữa 2 lần publish
    double rate = this->get_parameter("publish_rate").as_double();
    if (rate <= 0.0) rate = 50.0;
    min_publish_interval_ = 1.0 / rate;
    last_publish_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Target Publish Rate: %.1f Hz", rate);

    // 2. Kết nối Serial
    if (open_serial_port(port, baudrate)) {
      RCLCPP_INFO(this->get_logger(), "Connected to IMU on %s at %d", port.c_str(), baudrate);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
      return; 
    }

    // 3. Tạo Publishers (Reliable QoS)
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

    // 4. Timer đọc dữ liệu (Vẫn đọc nhanh để vét sạch buffer Serial)
    // Giữ nguyên 5ms (200Hz) để đọc buffer liên tục, tránh tràn
    timer_ = this->create_wall_timer(
      5ms, std::bind(&Adis16488Node::timer_callback, this));
  }

  ~Adis16488Node()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  int serial_fd_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string serial_buffer_;

  // --- THÊM: Biến kiểm soát tần số ---
  rclcpp::Time last_publish_time_;
  double min_publish_interval_;

  bool open_serial_port(const std::string &port, int baudrate)
  {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    speed_t speed;
    switch (baudrate) {
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      case 460800: speed = B460800; break; 
      case 921600: speed = B921600; break;
      default: speed = B460800;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB; 
    tty.c_cflag &= ~CSTOPB; 
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;    
    tty.c_cflag |= CREAD | CLOCAL; 
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
    tty.c_iflag &= ~(ICRNL | INLCR); 

    tcsetattr(serial_fd_, TCSANOW, &tty);
    return true;
  }

  void timer_callback()
  {
    if (serial_fd_ < 0) return;

    char buf[256];
    int n = read(serial_fd_, buf, sizeof(buf) - 1);

    if (n > 0) {
      buf[n] = '\0';
      serial_buffer_ += buf;
      process_buffer();
    }
  }

  void process_buffer()
  {
    size_t pos;
    while ((pos = serial_buffer_.find('\r')) != std::string::npos) {
      std::string line = serial_buffer_.substr(0, pos);
      serial_buffer_.erase(0, pos + 1);

      size_t start_pos = line.find('\n');
      if (start_pos != std::string::npos) {
        std::string data_str = line.substr(start_pos + 1);
        parse_and_publish(data_str);
      } else {
        parse_and_publish(line);
      }
    }
  }

  void parse_and_publish(const std::string &line)
  {
    // --- LOGIC GIỚI HẠN TẦN SỐ (THROTTLE) ---
    // Kiểm tra: Nếu chưa đủ thời gian kể từ lần gửi trước -> Bỏ qua packet này
    rclcpp::Time now = this->now();
    double time_diff = (now - last_publish_time_).seconds();

    if (time_diff < min_publish_interval_) {
        return; // Skip, không publish để giảm tải
    }

    // Nếu đủ thời gian thì tiếp tục xử lý
    std::stringstream ss(line);
    std::string segment;
    std::vector<double> values;

    while (std::getline(ss, segment, ' ')) {
      if (!segment.empty()) {
        try {
          values.push_back(std::stod(segment));
        } catch (...) {}
      }
    }

    if (values.size() >= 12) {
      // Cập nhật thời gian gửi lần cuối
      last_publish_time_ = now;

      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = now;
      imu_msg.header.frame_id = frame_id_;

      const double DEG_TO_RAD = M_PI / 180.0;
      const double G_TO_MS2   = 9.80665;

      // 1. Góc Roll, Pitch, Yaw
      double roll  = values[0] * 0.001 * DEG_TO_RAD;
      double pitch = -1.0 * values[1] * 0.001 * DEG_TO_RAD;
      double yaw   = -1.0 * values[2] * 0.001 * DEG_TO_RAD; 

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      imu_msg.orientation = tf2::toMsg(q);

      // 2. Vận tốc góc
      imu_msg.angular_velocity.x = values[3] * 0.001 * DEG_TO_RAD;
      imu_msg.angular_velocity.y = -1.0 * values[4] * 0.001 * DEG_TO_RAD;
      imu_msg.angular_velocity.z = -1.0 * values[5] * 0.001 * DEG_TO_RAD;

      // 3. Gia tốc tuyến tính
      imu_msg.linear_acceleration.x = values[6] * 0.0001 * G_TO_MS2;
      imu_msg.linear_acceleration.y = values[7] * 0.0001 * G_TO_MS2;
      imu_msg.linear_acceleration.z = values[8] * 0.0001 * G_TO_MS2;

      // Set covariance (Giữ nguyên)
      for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
      }
      imu_msg.orientation_covariance[0] = 0.01;
      imu_msg.orientation_covariance[4] = 0.01;
      imu_msg.orientation_covariance[8] = 0.01;
      
      imu_msg.angular_velocity_covariance[0] = 0.0001;
      imu_msg.angular_velocity_covariance[4] = 0.0001;
      imu_msg.angular_velocity_covariance[8] = 0.0001;

      imu_msg.linear_acceleration_covariance[0] = 0.001;
      imu_msg.linear_acceleration_covariance[4] = 0.001;
      imu_msg.linear_acceleration_covariance[8] = 0.001;

      imu_pub_->publish(imu_msg);

      // Publish Mag
      auto mag_msg = sensor_msgs::msg::MagneticField();
      mag_msg.header = imu_msg.header;  
      const double UNIT_TO_TESLA = 1e-8;

      mag_msg.magnetic_field.x = values[9]  * UNIT_TO_TESLA;
      mag_msg.magnetic_field.y = values[10] * UNIT_TO_TESLA;
      mag_msg.magnetic_field.z = -1.0 * values[11] * UNIT_TO_TESLA; 

      for (int i = 0; i < 9; i++) mag_msg.magnetic_field_covariance[i] = 0.0;
      mag_msg.magnetic_field_covariance[0] = 1e-8;
      
      mag_pub_->publish(mag_msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Adis16488Node>());
  rclcpp::shutdown();
  return 0;
}