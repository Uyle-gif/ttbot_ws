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
    this->declare_parameter("publish_rate", 50.0);

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    
    // Tính toán throttle
    double rate = this->get_parameter("publish_rate").as_double();
    if (rate <= 0.0) rate = 50.0;
    min_publish_interval_ = 1.0 / rate;
    last_publish_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Target Publish Rate: %.1f Hz", rate);
    RCLCPP_INFO(this->get_logger(), "--- CHE DO CALIB GYRO: KICH HOAT ---");
    RCLCPP_INFO(this->get_logger(), ">>> VUI LONG DE XE DUNG YEN TRONG 5 GIAY... <<<");

    // 2. Kết nối Serial
    if (open_serial_port(port, baudrate)) {
      RCLCPP_INFO(this->get_logger(), "Connected to IMU on %s at %d", port.c_str(), baudrate);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
      return; 
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_filtered", 10); 
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

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

  rclcpp::Time last_publish_time_;
  double min_publish_interval_;

  // --- Biến Calib Tự Động ---
  bool is_calibrating_ = true;
  int calib_count_ = 0;
  double gyro_z_bias_ = 0.0;
  const int CALIB_SAMPLES = 200; 

  // --- [THÊM MỚI] Biến tinh chỉnh thủ công (Manual Trim) ---
  // 0.125 độ/giây = 0.00218 rad/giây
  // Nếu trôi TRÁI -> Đặt số dương để trừ bớt.
  // Nếu trôi PHẢI -> Đặt số âm (trừ với trừ thành cộng).
  const double MANUAL_DRIFT_CORRECTION = (0.21667 + 0.0804597701 + 0.09130434782) * (M_PI / 180.0); 

  bool open_serial_port(const std::string &port, int baudrate)
  {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    speed_t speed;
    switch (baudrate) {
      case 460800: speed = B460800; break; 
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

    char buf[2048]; 
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

      if (line.length() < 10) continue; 

      size_t start_pos = line.find('\n');
      if (start_pos != std::string::npos) {
        parse_and_publish(line.substr(start_pos + 1));
      } else {
        parse_and_publish(line);
      }
    }
  }

  void parse_and_publish(const std::string &line)
  {
    rclcpp::Time now = this->now();
    double time_diff = (now - last_publish_time_).seconds();
    
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
      
      const double DEG_TO_RAD = M_PI / 180.0;
      const double G_TO_MS2   = 9.80665;

      // Đọc giá trị thô Gyro Z để xử lý
      double raw_gyro_z = -1.0 * values[5] * 0.001 * DEG_TO_RAD;

      // --- [LOGIC CALIB TỰ ĐỘNG] ---
      if (is_calibrating_) {
          gyro_z_bias_ += raw_gyro_z;
          calib_count_++;

          if (calib_count_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Calibrating... %d/%d", calib_count_, CALIB_SAMPLES);
          }

          if (calib_count_ >= CALIB_SAMPLES) {
              gyro_z_bias_ /= CALIB_SAMPLES;
              is_calibrating_ = false;
              RCLCPP_INFO(this->get_logger(), "AUTO CALIB DONE! Bias = %f", gyro_z_bias_);
              RCLCPP_INFO(this->get_logger(), "APPLYING MANUAL CORRECTION: %f rad/s", MANUAL_DRIFT_CORRECTION);
          }
          return; 
      }

      if (time_diff < min_publish_interval_) return;
      last_publish_time_ = now;

      // --- TẠO MESSAGE ---
      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = now;
      imu_msg.header.frame_id = frame_id_;

      // 1. Góc RPY
      double roll  = values[0] * 0.001 * DEG_TO_RAD;
      double pitch = -1.0 * values[1] * 0.001 * DEG_TO_RAD;
      double yaw   = -1.0 * values[2] * 0.001 * DEG_TO_RAD; 
      
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      imu_msg.orientation = tf2::toMsg(q);

      // 2. Vận tốc góc
      imu_msg.angular_velocity.x = values[3] * 0.001 * DEG_TO_RAD;
      imu_msg.angular_velocity.y = -1.0 * values[4] * 0.001 * DEG_TO_RAD;
      
      // --- [ĐIỂM QUAN TRỌNG NHẤT] ---
      // Công thức: Giá trị cuối = Raw - (Auto Bias) - (Manual Drift Correction)
      // Nếu vẫn trôi trái: Tăng MANUAL_DRIFT_CORRECTION lên
      // Nếu chuyển sang trôi phải: Giảm MANUAL_DRIFT_CORRECTION đi (hoặc đổi dấu)
      imu_msg.angular_velocity.z = raw_gyro_z - gyro_z_bias_ - MANUAL_DRIFT_CORRECTION; 

      // 3. Gia tốc
      imu_msg.linear_acceleration.x = values[6] * 0.0001 * G_TO_MS2;
      imu_msg.linear_acceleration.y = -1.0 * values[7] * 0.0001 * G_TO_MS2;
      imu_msg.linear_acceleration.z = values[8] * 0.0001 * G_TO_MS2;

      // Covariance
      imu_msg.orientation_covariance[8] = 0.05;
      imu_msg.angular_velocity_covariance[8] = 0.00001; 
      imu_msg.linear_acceleration_covariance[8] = 0.001;

      imu_pub_->publish(imu_msg);

      // Mag (đã comment để tập trung vào Gyro)
      /*
      auto mag_msg = sensor_msgs::msg::MagneticField();
      ...
      mag_pub_->publish(mag_msg);
      */
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