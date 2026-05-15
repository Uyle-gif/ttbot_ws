#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class StampedTwistMux : public rclcpp::Node
{
public:
  StampedTwistMux()
  : Node("stamped_twist_mux")
  {
    this->declare_parameter("joy_timeout", 0.5);
    joy_timeout_ = this->get_parameter("joy_timeout").as_double();

    RCLCPP_INFO(this->get_logger(), "--> MUX STARTED. Waiting for inputs...");

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_cmd_out", 10);

    sub_joy_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "joy_cmd_vel", 10,
      std::bind(&StampedTwistMux::joyCallback, this, std::placeholders::_1));

    sub_mpc_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "mpc_cmd_vel", 10,
      std::bind(&StampedTwistMux::mpcCallback, this, std::placeholders::_1));

    last_joy_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

private:
  void joyCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    
    
    last_joy_time_ = this->now();
    pub_cmd_->publish(*msg);
  }

  void mpcCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    auto current_time = this->now();
    double time_diff = (current_time - last_joy_time_).seconds();

    if (time_diff > joy_timeout_) {
        pub_cmd_->publish(*msg);
    } else {
        // [DEBUG] In ra để biết MPC bị chặn
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MPC BLOCKED by Joystick!");
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_joy_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_mpc_;
  rclcpp::Time last_joy_time_;
  double joy_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StampedTwistMux>());
  rclcpp::shutdown();
  return 0;
}