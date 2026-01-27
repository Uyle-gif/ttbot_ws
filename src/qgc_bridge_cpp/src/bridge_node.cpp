#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <thread>

// Linux Socket Headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

// MAVLink Headers
#include "mavlink/ardupilotmega/mavlink.h"

using namespace std::chrono_literals;

// Hằng số bán kính trái đất
const double EARTH_RADIUS = 6378137.0;

class QGCBridgeNode : public rclcpp::Node
{
public:
    QGCBridgeNode() : Node("qgc_bridge_node")
    {
        // [THÊM MỚI] --- ĐỌC THAM SỐ OFFSET TỪ LAUNCH FILE ---
        this->declare_parameter("heading_offset_deg", 0.0); // Mặc định  
        double offset_deg = this->get_parameter("heading_offset_deg").as_double();
        heading_offset_rad_ = offset_deg * M_PI / 180.0;     // Đổi sang Radian

        RCLCPP_INFO(this->get_logger(), "--- QGC BRIDGE C++ (SYSID=%d) ---", sys_id_);
        RCLCPP_INFO(this->get_logger(), "Heading Offset: %.2f deg", offset_deg);
        // ----------------------------------------------------

        // 1. Khởi tạo UDP Socket
        setup_udp_socket(14551);
        boot_time_start_ = std::chrono::steady_clock::now();

        // 2. Khởi tạo Parameter (Giống Python)
        init_parameters();

        // 3. Tạo Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
        pid_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid_all_tuning", 10);
        mpc_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mpc_tuning", 10); 
        arm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/system/armed", 10);

        // 4. Tạo Subscribers
        // Dùng QoS Best Effort cho IMU để giảm lag
        rclcpp::QoS qos_sensor(10);
        qos_sensor.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&QGCBridgeNode::gps_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/global", 10, std::bind(&QGCBridgeNode::odom_callback, this, std::placeholders::_1));

        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     "/imu/data_filtered", qos_sensor, std::bind(&QGCBridgeNode::imu_callback, this, std::placeholders::_1));

        // 5. Timers (Thay thế luồng)
        // Timer đọc dữ liệu UDP (chạy cực nhanh 100Hz - 10ms)
        read_timer_ = this->create_wall_timer(
            10ms, std::bind(&QGCBridgeNode::read_mavlink_loop, this));
        
        // Timer gửi Heartbeat (1Hz)
        hb_timer_ = this->create_wall_timer(
            1000ms, std::bind(&QGCBridgeNode::send_heartbeat, this));

        // Timer gửi Sys Status & Autopilot Version (1Hz & 2Hz)
        status_timer_ = this->create_wall_timer(
            1000ms, std::bind(&QGCBridgeNode::send_sys_status, this));
        
        ver_timer_ = this->create_wall_timer(
            2000ms, std::bind(&QGCBridgeNode::send_autopilot_version, this));

        // Gửi PID lần đầu
        publish_pid_to_ros();
        publish_mpc_to_ros(); 
    }

    ~QGCBridgeNode() {
        close(sock_fd_);
    }

private:
    //  [THÊM MỚI ĐOẠN NÀY] -------------------------
    uint8_t target_sys_id_ = 0;       // Lưu ID của QGC
    uint8_t target_comp_id_ = 0;      // Lưu Component của QGC
    uint8_t current_mission_type_ = 0; // Lưu loại nhiệm vụ (Mission/Rally)
    // ---------------------------------------------    
    // --- Variables ---
    int sys_id_ = 1;
    int comp_id_ = 1;
    int sock_fd_;
    struct sockaddr_in loc_addr_;
    struct sockaddr_in rem_addr_;
    socklen_t rem_addr_len_ = sizeof(rem_addr_);
    
    std::chrono::steady_clock::time_point boot_time_start_;
    
    // State variables
    bool is_armed_ = false;
    bool has_orientation_ = false;
    bool home_set_ = false;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    double heading_offset_rad_ = 0.0; // Đặt về 0, giá trị thật sẽ lấy ở trên Constructor

    // Mission handling
    int mission_count_ = 0;
    int current_wp_seq_ = 0;
    nav_msgs::msg::Path current_path_;

    // Parameters
    std::map<std::string, float> param_dict_;
    std::vector<std::string> param_keys_;

    // ROS Handles
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_pub_; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr read_timer_, hb_timer_, status_timer_, ver_timer_;

    // --- Helper Functions ---

    uint32_t get_boot_time_ms() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - boot_time_start_).count();
    }

    void setup_udp_socket(int port) {
        sock_fd_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        memset(&loc_addr_, 0, sizeof(loc_addr_));
        loc_addr_.sin_family = AF_INET;
        loc_addr_.sin_addr.s_addr = INADDR_ANY;
        loc_addr_.sin_port = htons(port);

        if (bind(sock_fd_, (struct sockaddr *)&loc_addr_, sizeof(loc_addr_)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed. Port %d busy?", port);
            return;
        }

        // Set Non-blocking
        fcntl(sock_fd_, F_SETFL, O_NONBLOCK | FASYNC);
        
        // Mặc định gửi về localhost nếu chưa nhận được gói tin nào
        memset(&rem_addr_, 0, sizeof(rem_addr_));
        rem_addr_.sin_family = AF_INET;
        rem_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");
        rem_addr_.sin_port = htons(14550);
    }

    // --- [THÊM MỚI] Hàm tính khoảng cách ---
    double calculate_distance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // --- [THÊM MỚI] Hàm nội suy tuyến tính ---
    void interpolate_and_add_points(double start_x, double start_y, double end_x, double end_y) {
        const double POINT_DENSITY = 0.2; // Mật độ: 20cm/điểm
        
        double dist = calculate_distance(start_x, start_y, end_x, end_y);
        
        // Nếu điểm quá gần thì chỉ thêm điểm đích
        if (dist < POINT_DENSITY) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = end_x;
            pose.pose.position.y = end_y;
            current_path_.poses.push_back(pose);
            return;
        }

        // Chia nhỏ quãng đường
        int num_points = std::floor(dist / POINT_DENSITY);
        double step_x = (end_x - start_x) / num_points;
        double step_y = (end_y - start_y) / num_points;

        for (int i = 1; i <= num_points; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = start_x + (step_x * i);
            pose.pose.position.y = start_y + (step_y * i);
            current_path_.poses.push_back(pose);
        }
    }

    void send_mavlink_message(mavlink_message_t* msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
        // Gửi tới địa chỉ remote đã lưu (QGC)
        sendto(sock_fd_, buf, len, 0, (struct sockaddr *)&rem_addr_, sizeof(rem_addr_));
    }

    void init_parameters() {
        param_dict_["SYSID_THISMAV"] = (float)sys_id_;
        param_dict_["PILOT_THR_BHV"] = 0.0;
        param_dict_["ARMING_CHECK"] = 0.0;
        param_dict_["VEL1_KP"] = 1.5; param_dict_["VEL1_KI"] = 0.01; param_dict_["VEL1_KD"] = 0.5;
        param_dict_["VEL2_KP"] = 1.5; param_dict_["VEL2_KI"] = 0.01; param_dict_["VEL2_KD"] = 0.5;
        param_dict_["POS_KP"] = 0.8;  param_dict_["POS_KI"] = 0.00; param_dict_["POS_KD"] = 0.05;
        param_dict_["MAX_SPEED"] = 2.0;
        // MPC
        param_dict_["MPC_SPEED"]   = 1.5f;  // Desired Speed
        param_dict_["MPC_NP"]      = 20.0f; // Horizon Steps (Int nhưng truyền Float)
        param_dict_["MPC_DT"]      = 0.1f;  // Time Step
        param_dict_["MPC_Q_EY"]    = 10.0f; // CTE Weight
        param_dict_["MPC_Q_EPSI"]  = 8.0f;  // Heading Error Weight
        param_dict_["MPC_R_DELTA"] = 20.0f; // Steering Effort Weight          

        for(auto const& [key, val] : param_dict_) {
            param_keys_.push_back(key);
        }
    }

    void publish_pid_to_ros() {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {
            param_dict_["VEL1_KP"], param_dict_["VEL1_KI"], param_dict_["VEL1_KD"],
            param_dict_["VEL2_KP"], param_dict_["VEL2_KI"], param_dict_["VEL2_KD"],
            param_dict_["POS_KP"],  param_dict_["POS_KI"],  param_dict_["POS_KD"]
        };
        pid_pub_->publish(msg);
    }

    void publish_mpc_to_ros() {
    auto msg = std_msgs::msg::Float32MultiArray();
    // Quy ước thứ tự: [0]:Speed, [1]:Np, [2]:dt, [3]:Q_ey, [4]:Q_epsi, [5]:R_delta
    msg.data = {
        param_dict_["MPC_SPEED"],
        param_dict_["MPC_NP"],
        param_dict_["MPC_DT"],
        param_dict_["MPC_Q_EY"],
        param_dict_["MPC_Q_EPSI"],
        param_dict_["MPC_R_DELTA"]
    };
    mpc_pub_->publish(msg);
    }

    // --- Periodic Tasks ---

    void send_heartbeat() {
        mavlink_message_t msg;
        uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        if (is_armed_) base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

        mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg,
            MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC, base_mode, 0, MAV_STATE_ACTIVE);
        send_mavlink_message(&msg);
    }

    void send_sys_status() {
        mavlink_message_t msg;
        uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_GPS | 
                           MAV_SYS_STATUS_SENSOR_BATTERY | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_PREARM_CHECK;
        
        // SỬA LỖI: Thêm 3 số 0 vào cuối hàm (cho extended_onboard_control_sensors_present/enabled/health)
        mavlink_msg_sys_status_pack(sys_id_, comp_id_, &msg,
            sensors, sensors, sensors, 500, 12600, 500, 99, 
            0, 0, 0, 0, 0, 0, 
            0, 0, 0); // <--- Thêm 3 tham số này
        
        send_mavlink_message(&msg);
    }

    void send_autopilot_version() {
        mavlink_message_t msg;
        uint64_t caps = MAV_PROTOCOL_CAPABILITY_MISSION_INT | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                        MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
        uint8_t custom[8] = {0};
        mavlink_msg_autopilot_version_pack(sys_id_, comp_id_, &msg,
            caps, 1, 1, 1, 1, custom, custom, custom, 0, 0, 0, custom);
        send_mavlink_message(&msg);
    }



    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // --- PHẦN 1: PHÂN TÍCH TRẠNG THÁI GPS (QUAN TRỌNG NHẤT) ---
        // Chuẩn ROS (NavSatFix):
        // status = -1: Không có sóng (STATUS_NO_FIX)
        // status =  0: Đã Fix (STATUS_FIX) - Gazebo và GPS thường đều trả về cái này
        // status =  1: SBAS (Tốt hơn)
        // status =  2: GBAS (Tốt nhất)
        
        uint8_t mav_fix_type;
        uint8_t satellites_visible;

        if (msg->status.status >= 0) {
            // TRƯỜNG HỢP TỐT (Dùng cho cả Sim và Real khi có sóng)
            mav_fix_type = 3;   // GPS_FIX_TYPE_3D_FIX (Mã 3)
            satellites_visible = 12; // NavSatFix không có số vệ tinh, ta giả lập 12 để QGC vui
        } else {
            // TRƯỜNG HỢP MẤT SÓNG (Chỉ xảy ra ở Real World hoặc Sim lỗi)
            mav_fix_type = 1;   // GPS_FIX_TYPE_NO_FIX (Mã 1)
            satellites_visible = 0;
        }

        // --- PHẦN 2: CHUẨN BỊ DỮ LIỆU ---
        uint32_t boot_time = get_boot_time_ms();
        int32_t lat_int = (int32_t)(msg->latitude * 1e7);
        int32_t lon_int = (int32_t)(msg->longitude * 1e7);
        int32_t alt_int = (int32_t)(msg->altitude * 1000);

        // --- PHẦN 3: SET HOME (Chỉ set khi GPS TỐT lần đầu tiên) ---
        if (!home_set_ && mav_fix_type == 3) {
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            home_set_ = true;
            RCLCPP_INFO(this->get_logger(), "HOME SET: %f, %f (Status: %d)", 
                        origin_lat_, origin_lon_, msg->status.status);
            
            mavlink_message_t mav_msg;
            float q[4] = {0};
            mavlink_msg_home_position_pack(sys_id_, comp_id_, &mav_msg,
                lat_int, lon_int, alt_int,
                0, 0, 0, q, 0, 0, 0, 0);
            send_mavlink_message(&mav_msg);
        }

        
        // --- PHẦN 5: GỬI GPS_RAW_INT (Trạng thái vệ tinh) ---
        // Đây là gói tin quyết định QGC báo "No GPS Lock" hay "3D Lock"
        mavlink_message_t msg_raw;
        mavlink_msg_gps_raw_int_pack(sys_id_, comp_id_, &msg_raw,
            boot_time,
            mav_fix_type,       // <--- Tự động đổi giữa 1 (No Fix) và 3 (3D Fix)
            lat_int, lon_int, alt_int,
            0xFFFF, 0xFFFF,     // HDOP, VDOP (Không có dữ liệu thì để max)
            0,                  // Velocity
            0xFFFF,             // Course
            satellites_visible, // <--- 0 hoặc 12 tùy tình trạng
            0, 0, 0, 0, 0, 0);
        send_mavlink_message(&msg_raw);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Chỉ xử lý khi đã biết điểm Home (để tính offset Lat/Lon)
        if (!home_set_) return;

        // --- 1. Chuyển đổi Vị trí (X, Y map -> Lat, Lon) ---
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double lat0_rad = origin_lat_ * M_PI / 180.0;
        double dLat = y / EARTH_RADIUS;
        double dLon = x / (EARTH_RADIUS * std::cos(lat0_rad));

        double current_lat = origin_lat_ + (dLat * 180.0 / M_PI);
        double current_lon = origin_lon_ + (dLon * 180.0 / M_PI);

        // --- 2. Xử lý Quaternion sang Euler (Roll, Pitch, Yaw) ---
        // [PHẦN MỚI] Tính đủ 3 góc để gửi cho QGC
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        // Tính Roll
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        double odom_roll = std::atan2(sinr_cosp, cosr_cosp);

        // Tính Pitch
        double sinp = 2 * (qw * qy - qz * qx);
        double odom_pitch = 0.0;
        if (std::abs(sinp) >= 1)
            odom_pitch = std::copysign(M_PI / 2, sinp);
        else
            odom_pitch = std::asin(sinp);

        // Tính Yaw
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double raw_yaw = std::atan2(siny_cosp, cosy_cosp);

        // --- 3. Áp dụng Offset ---
        double final_yaw = -raw_yaw + heading_offset_rad_; 

        // Chuẩn hóa -PI ... +PI
        while (final_yaw > M_PI) final_yaw -= 2 * M_PI;
        while (final_yaw < -M_PI) final_yaw += 2 * M_PI;

        // Đổi sang độ (0-360) cho Global Position
        double deg = final_yaw * 180.0 / M_PI;
        if (deg < 0) deg += 360.0;
        uint16_t heading_cdeg = (uint16_t)(deg * 100);

        // --- 4. GỬI TIN NHẮN ATTITUDE (MSG #30) ---
        // [QUAN TRỌNG] Đây là phần giúp mũi tên trên QGC xoay
        mavlink_message_t msg_att;
        mavlink_msg_attitude_pack(sys_id_, comp_id_, &msg_att,
            get_boot_time_ms(),
            odom_roll,   
            odom_pitch,  
            final_yaw,   // Yaw đã chỉnh offset
            msg->twist.twist.angular.x, 
            msg->twist.twist.angular.y, 
            msg->twist.twist.angular.z);
        send_mavlink_message(&msg_att);

        // --- 5. GỬI TIN NHẮN VỊ TRÍ (MSG #33) ---
        mavlink_message_t mav_msg;
        mavlink_msg_global_position_int_pack(sys_id_, comp_id_, &mav_msg,
            get_boot_time_ms(),
            (int32_t)(current_lat * 1e7), (int32_t)(current_lon * 1e7),
            (int32_t)(msg->pose.pose.position.z * 1000) + 10000, 
            10000, // Relative Alt
            (int16_t)(msg->twist.twist.linear.x * 100), 
            (int16_t)(msg->twist.twist.linear.y * 100), 
            (int16_t)(msg->twist.twist.linear.z * 100),
            heading_cdeg);
        send_mavlink_message(&mav_msg);
    }

    // --- MAVLink Reading Loop ---

    void read_mavlink_loop() {
        char buf[2048];
        struct sockaddr_in src_addr;
        socklen_t addr_len = sizeof(src_addr);
        
        ssize_t recv_len = recvfrom(sock_fd_, buf, sizeof(buf), 0, (struct sockaddr *)&src_addr, &addr_len);

        if (recv_len > 0) {
            // Cập nhật địa chỉ gửi về (để trả lời đúng IP của QGC)
            rem_addr_ = src_addr; 
            
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < recv_len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    handle_message(msg);
                }
            }
        }
    }

    void handle_message(const mavlink_message_t& msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_PARAM_SET:
                handle_param_set(msg);
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                send_all_params();
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                handle_command_long(msg);
                break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
                handle_mission_count(msg);
                break;
            case MAVLINK_MSG_ID_MISSION_ITEM_INT:
                handle_mission_item(msg);
                break;
            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                current_path_ = nav_msgs::msg::Path();
                current_path_.header.frame_id = "map";
                path_pub_->publish(current_path_);
                send_mission_ack();
                break;     
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: // SỬA LỖI MISSION FAILED
                handle_mission_request_list(msg);
                break;     
            default:
                break;
        }
    }

    // [ĐÃ SỬA - CÁCH MỚI] Sử dụng Struct để đóng gói tin nhắn (An toàn hơn)
    void handle_mission_request_list(const mavlink_message_t& msg) {
        mavlink_message_t ack_msg;
        mavlink_mission_count_t mcount; // Tạo struct chứa dữ liệu
        
        // Gán từng trường dữ liệu rõ ràng
        mcount.target_system = msg.sysid;
        mcount.target_component = msg.compid;
        mcount.count = 0; // Số lượng nhiệm vụ = 0
        mcount.mission_type = MAV_MISSION_TYPE_MISSION;
        
        // Dùng hàm encode: Chỉ cần truyền 4 tham số chuẩn
        mavlink_msg_mission_count_encode(sys_id_, comp_id_, &ack_msg, &mcount);
            
        send_mavlink_message(&ack_msg);
    }

    void handle_param_set(const mavlink_message_t& msg) {
        mavlink_param_set_t pset;
        mavlink_msg_param_set_decode(&msg, &pset);
        
        // Chuẩn hóa chuỗi ID
        char key_buf[17];
        strncpy(key_buf, pset.param_id, 16);
        key_buf[16] = '\0';
        std::string key(key_buf);

        if (param_dict_.find(key) != param_dict_.end()) {
            param_dict_[key] = pset.param_value;
            
            // Gửi lại Param Value xác nhận
            mavlink_message_t ack_msg;
            // Tìm index
            int idx = 0;
            for(size_t i=0; i<param_keys_.size(); ++i) if(param_keys_[i] == key) idx = i;

            mavlink_msg_param_value_pack(sys_id_, comp_id_, &ack_msg,
                pset.param_id, pset.param_value, MAV_PARAM_TYPE_REAL32, param_keys_.size(), idx);
            send_mavlink_message(&ack_msg);

            // Nếu là param PID thì pub ra ROS
            if (key.find("VEL") != std::string::npos || key.find("POS") != std::string::npos) {
                publish_pid_to_ros();
            }
            // [THÊM MỚI] Kiểm tra nếu là MPC
            else if (key.find("MPC") != std::string::npos) {
                publish_mpc_to_ros();
                RCLCPP_INFO(this->get_logger(), "MPC Params Updated from QGC: %s = %.2f", key.c_str(), pset.param_value);
            }
            
        }
    }

    void send_all_params() {
        for (size_t i = 0; i < param_keys_.size(); ++i) {
            std::string key = param_keys_[i];
            mavlink_message_t msg;
            mavlink_msg_param_value_pack(sys_id_, comp_id_, &msg,
                key.c_str(), param_dict_[key], MAV_PARAM_TYPE_REAL32, param_keys_.size(), i);
            send_mavlink_message(&msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // Delay nhỏ để tránh flood
        }
    }

    void handle_command_long(const mavlink_message_t& msg) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        mavlink_message_t ack;
        mavlink_msg_command_ack_pack(sys_id_, comp_id_, &ack, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, 0, 0);

        if (cmd.command == 520) { // REQUEST_AUTOPILOT_CAPABILITIES
            send_mavlink_message(&ack);
            send_autopilot_version();
        } else if (cmd.command == 519) { // REQUEST_PROTOCOL_VERSION
            send_mavlink_message(&ack);
            mavlink_message_t proto_msg;
            mavlink_msg_protocol_version_pack(sys_id_, comp_id_, &proto_msg, 200, 200, 200, NULL, NULL);
            send_mavlink_message(&proto_msg);
        } else if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) { // 400
            bool new_state = (cmd.param1 == 1.0f);
            
            // Xử lý chuyển trạng thái
            if (new_state && !is_armed_) {
                // --> Chuyển sang ARMED
                is_armed_ = true;
                RCLCPP_INFO(this->get_logger(), "System ARMED. Executing stored mission...");
                
                // Nếu đã có mission lưu sẵn thì gửi đi ngay
                if (!current_path_.poses.empty()) {
                    current_path_.header.stamp = this->now();
                    path_pub_->publish(current_path_);
                }
            } 
            else if (!new_state && is_armed_) {
                // --> Chuyển sang DISARMED
                is_armed_ = false;
                RCLCPP_INFO(this->get_logger(), "System DISARMED. Stopping...");
                
                // Gửi path rỗng để xe dừng lại (Safety)
                nav_msgs::msg::Path empty_path;
                empty_path.header.frame_id = "map";
                empty_path.header.stamp = this->now();
                path_pub_->publish(empty_path);
            }

            // Publish trạng thái Arm ra ROS
            std_msgs::msg::Bool arm_msg;
            arm_msg.data = is_armed_;
            arm_pub_->publish(arm_msg);
            
            send_mavlink_message(&ack);
            send_heartbeat();
        }
    }

    void handle_mission_count(const mavlink_message_t& msg) {
        mavlink_mission_count_t mcount;
        mavlink_msg_mission_count_decode(&msg, &mcount);

        // --- [SỬA ĐỔI BẮT ĐẦU] ---
        // 1. Lưu lại ai là người gửi (QGC) để tí nữa gửi trả lời cho đúng người
        target_sys_id_ = msg.sysid;
        target_comp_id_ = msg.compid;
        
        // 2. Lưu lại loại nhiệm vụ (Mission, Fence, hay Rally?)
        current_mission_type_ = mcount.mission_type; 
        // -------------------------

        mission_count_ = mcount.count;
        current_wp_seq_ = 0;
        
        // --- [SỬA ĐỔI BẮT ĐẦU] ---
        // 3. Chỉ xóa đường dẫn cũ nếu đây là nhiệm vụ bay (Mission)
        // Nếu QGC chỉ gửi Rally Point (Điểm an toàn), ta không nên xóa đường đi chính
        if (current_mission_type_ == MAV_MISSION_TYPE_MISSION) {
            current_path_ = nav_msgs::msg::Path();
            current_path_.header.frame_id = "map";
        }
        // -------------------------

        // Request WP 0
        send_mission_request(0);
    }

    void handle_mission_item(const mavlink_message_t& msg) {
        mavlink_mission_item_int_t item;
        mavlink_msg_mission_item_int_decode(&msg, &item);

        if (item.seq == current_wp_seq_) {
            
            // --- [SỬA ĐỔI BẮT ĐẦU] ---
            // Thêm điều kiện: Chỉ xử lý tọa độ nếu là MISSION_TYPE_MISSION
            if (current_mission_type_ == MAV_MISSION_TYPE_MISSION) {
                if (item.command == MAV_CMD_NAV_WAYPOINT && home_set_) {
                    // 1. Tính tọa độ điểm đích (Target Point) từ GPS
                    double wp_lat = item.x / 1e7;
                    double wp_lon = item.y / 1e7;
                    
                    double lat0_rad = origin_lat_ * M_PI / 180.0;
                    double dLat = (wp_lat - origin_lat_) * M_PI / 180.0;
                    double dLon = (wp_lon - origin_lon_) * M_PI / 180.0;

                    double target_x = dLon * EARTH_RADIUS * std::cos(lat0_rad);
                    double target_y = dLat * EARTH_RADIUS;

                    

                    // Kiểm tra: Đây có phải là điểm đầu tiên của nhiệm vụ không?
                    if (current_path_.poses.empty()) {
                        // TRƯỜNG HỢP ĐIỂM ĐẦU TIÊN:
                        // KHÔNG vẽ đường từ (0,0). 
                        // Chỉ thêm đúng điểm đích vào danh sách.
                        // MPC Controller sẽ tự động tính đường từ vị trí hiện tại của xe đến điểm này.
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header.frame_id = "map";
                        pose.pose.position.x = target_x;
                        pose.pose.position.y = target_y;
                        current_path_.poses.push_back(pose);
                    } 
                    else {
                        // TRƯỜNG HỢP CÁC ĐIỂM TIẾP THEO (WP 2, WP 3...):
                        // Lấy điểm cuối cùng của đoạn trước làm điểm bắt đầu cho đoạn này
                        // để đảm bảo đường đi liền mạch.
                        double start_x = current_path_.poses.back().pose.position.x;
                        double start_y = current_path_.poses.back().pose.position.y;
                        
                        // Gọi hàm chia nhỏ điểm
                        interpolate_and_add_points(start_x, start_y, target_x, target_y);
                    }
                }
            }
            // -------------------------

            current_wp_seq_++;
            if (current_wp_seq_ < mission_count_) {
                send_mission_request(current_wp_seq_);
            } else {
                send_mission_ack();
                
                // --- [SỬA ĐỔI BẮT ĐẦU] ---
                // Chỉ publish path nếu là Mission Type
                if (current_mission_type_ == MAV_MISSION_TYPE_MISSION) {
                    current_path_.header.stamp = this->now();
                    
                    if (is_armed_) {
                        path_pub_->publish(current_path_);
                        RCLCPP_INFO(this->get_logger(), "Mission Updated (ARMED). Sending to controller.");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Mission Stored (%d points). Waiting for ARM command...", (int)current_path_.poses.size());
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Received Non-Mission data (Type: %d). Acknowledged but ignored.", current_mission_type_);
                }
                // -------------------------
            }
        }
    }

    void send_mission_request(int seq) {
        mavlink_message_t msg;
        
        // --- [SỬA ĐỔI BẮT ĐẦU] ---
        // Thay tham số thứ 4, 5 bằng target_sys_id_, target_comp_id_
        // Thay tham số cuối cùng bằng current_mission_type_
        mavlink_msg_mission_request_int_pack(sys_id_, comp_id_, &msg, 
                                             target_sys_id_,   // <--- Gửi đến QGC
                                             target_comp_id_,  // <--- Gửi đến QGC
                                             seq, 
                                             current_mission_type_); // <--- Đúng loại type (Mission/Rally)
        // -------------------------
        
        send_mavlink_message(&msg);
    }

    void send_mission_ack() {
        mavlink_message_t msg;
        
        // --- [SỬA ĐỔI BẮT ĐẦU] ---
        // Thay tham số thứ 4, 5 bằng target_sys_id_, target_comp_id_
        // Thay tham số mission_type bằng current_mission_type_
        mavlink_msg_mission_ack_pack(sys_id_, comp_id_, &msg, 
                                     target_sys_id_,    // <--- Gửi trả cho QGC (QUAN TRỌNG)
                                     target_comp_id_,   // <--- Gửi trả cho QGC
                                     MAV_MISSION_ACCEPTED, 
                                     current_mission_type_, // <--- Xác nhận đúng loại type
                                     0); 
        // -------------------------

        send_mavlink_message(&msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Với C++, Executor cơ bản cũng đủ vì ta dùng Timer non-blocking,
    // nhưng dùng MultiThreadedExecutor vẫn tốt cho hiệu năng.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<QGCBridgeNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}