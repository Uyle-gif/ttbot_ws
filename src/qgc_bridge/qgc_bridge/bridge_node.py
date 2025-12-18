import os
# [BẮT BUỘC] Phải đặt biến môi trường này TRƯỚC khi import pymavlink
# Để kích hoạt tính năng mở rộng (uid2) của MAVLink 2.0
os.environ['MAVLINK20'] = '1'

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatFix, Imu # [NEW] Thêm Imu msg
from std_msgs.msg import Float32MultiArray, Bool
import math
import time

class QGCBridge(Node):
    def __init__(self):
        super().__init__('qgc_bridge_node')
        
        # [CẤU HÌNH] Quay về ID=1 (Chuẩn mặc định của ArduPilot)
        # Vì code đã fix lỗi nhận diện nên không cần dùng ID=3 để né cache nữa
        self.sys_id = 1
        self.comp_id = 1
        self.connection_string = 'udpout:localhost:14550' 
        self.heading_offset_rad = math.pi / -2  # <--- CHỈNH GÓC Ở ĐÂY
        # Dialect ardupilotmega để hỗ trợ đầy đủ các lệnh của ArduRover
        self.mav = mavutil.mavlink_connection(self.connection_string, source_system=self.sys_id, source_component=self.comp_id, dialect='ardupilotmega')
        
        # [FIX TIME] Lưu mốc thời gian khởi động
        self.boot_time = time.time()
        
        self.get_logger().info(f"--- BRIDGE ULTIMATE FIXED (SYSID={self.sys_id}) ---")

        # --- ROS 2 PUBLISHERS ---
        self.path_pub = self.create_publisher(Path, '/mpc_path', 10)
        self.params_pub = self.create_publisher(Float32MultiArray, '/tuning/pid_params', 10)
        self.arm_pub = self.create_publisher(Bool, '/system/armed', 10)

        # --- ROS 2 SUBSCRIBERS ---
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/data_filtered', self.imu_callback, 10)

        # Biến quản lý
        self.mission_count = 0
        self.current_wp_seq = 0
        self.is_armed = False 
        self.last_odom_time = 0.0

        # [NEW] Biến lưu góc quay hiện tại (Radian)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.has_orientation = False # Cờ đánh dấu đã có dữ liệu hướng

        # Kho tham số giả lập
        self.param_dict = {
            'SYSID_THISMAV': float(self.sys_id),
            'PILOT_THR_BHV': 0.0, 
            'FS_GCS_ENABLE': 0.0, 'FS_OPTIONS': 0.0, 'FS_GCS_TIMEOUT': 5.0, 'FS_ACTION': 0.0,
            'ARMING_CHECK': 0.0, # Tắt check để luôn Ready
            'VEL_KP': 1.5, 'VEL_KI': 0.01, 'VEL_KD': 0.5, 'MAX_SPEED': 2.0
        }
        self.param_keys = list(self.param_dict.keys())

        self.origin_lat = None
        self.origin_lon = None
        self.current_path = Path() 

        # --- TIMERS ---
        self.create_timer(1.0, self.send_heartbeat)      # 1Hz Heartbeat
        self.create_timer(0.01, self.read_mavlink_loop)  # Xử lý tin nhắn đến
        self.create_timer(1.0, self.send_sys_status)     # 1Hz Status
        self.create_timer(2.0, self.send_autopilot_version) # 0.5Hz Gửi định danh (QUAN TRỌNG)

    # [HÀM FIX TRÀN SỐ]
    def get_boot_time(self):
        return int((time.time() - self.boot_time) * 1000)

    def send_heartbeat(self):
        # Base mode: MANUAL + ARMED/DISARMED
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        if self.is_armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER,       # Loại xe
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, # Hãng
            base_mode,
            0, # Custom mode = 0 (Manual)
            mavutil.mavlink.MAV_STATE_ACTIVE)            # Active = Ready

    def send_sys_status(self):
        # Báo cáo toàn bộ cảm biến Healthy
        sensors_health = (
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS |
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY |
            mavutil.mavlink.MAV_SYS_STATUS_AHRS | 
            mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        )
        self.mav.mav.sys_status_send(
            sensors_health, sensors_health, sensors_health,
            200, 12600, 500, 99, 0, 0, 0, 0, 0, 0
        )

    # [HÀM ĐÃ SỬA LỖI TYPE ERROR]
    def send_autopilot_version(self):
        capabilities = (
            mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_INT | 
            mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
            mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
            mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
        )
        # Truyền đúng 12 tham số theo chuẩn pymavlink
        self.mav.mav.autopilot_version_send(
            capabilities, 
            1, 1, 1, 1,                  # 4 version numbers (sw, mid, os, board)
            bytes([0]*8),                # flight_custom_version (8 bytes)
            bytes([0]*8),                # middleware_custom_version (8 bytes)
            bytes([0]*8),                # os_custom_version (8 bytes)
            0, 0,                        # vendor_id, product_id
            0,                           # uid
            bytes([0]*18)                # uid2 (18 bytes) - Chỉ có ở MAVLink 2
        )
    # [FIX HƯỚNG] IMU Callback có áp dụng Offset
    def imu_callback(self, msg):
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Áp dụng Offset tại đây
        self.yaw = raw_yaw + self.heading_offset_rad
        
        # Chuẩn hóa về -pi đến pi
        if self.yaw > math.pi: self.yaw -= 2*math.pi
        elif self.yaw < -math.pi: self.yaw += 2*math.pi
        
        self.has_orientation = True

        self.mav.mav.attitude_send(
            self.get_boot_time(), self.roll, self.pitch, self.yaw,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        )

    def gps_callback(self, msg):
        if self.origin_lat is None and msg.status.status >= 0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"HOME SET: {self.origin_lat}, {self.origin_lon}")
            self.mav.mav.home_position_send(
                int(msg.latitude * 1e7), int(msg.longitude * 1e7), int(msg.altitude * 1000),
                0, 0, 0, [0,0,0,0], 0, 0, 0)

        # Smart Switch: Chỉ gửi GPS nếu Odom bị mất quá 1s
        time_since_last_odom = time.time() - self.last_odom_time
        if msg.status.status >= 0 and time_since_last_odom > 1.0:
            self.mav.mav.global_position_int_send(
                self.get_boot_time(),
                int(msg.latitude * 1e7), int(msg.longitude * 1e7),
                int(msg.altitude * 1000), int(10 * 1000),
                0, 0, 0, 65535
            )

    def odom_callback(self, msg):
        if self.origin_lat is None: return
        self.last_odom_time = time.time()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        R = 6378137.0
        lat0_rad = math.radians(self.origin_lat)
        dLat = y / R
        dLon = x / (R * math.cos(lat0_rad))
        current_lat = self.origin_lat + math.degrees(dLat)
        current_lon = self.origin_lon + math.degrees(dLon)
        
        # [UPDATE] Tính heading. Ưu tiên dùng IMU nếu có, nếu không thì dùng Odom
        heading_cdeg = 0
        if self.has_orientation:
            # Dùng Yaw từ IMU (Chính xác hơn)
            deg = math.degrees(self.yaw)
            if deg < 0: deg += 360
            heading_cdeg = int(deg * 100)
        else:
            # Fallback: Tính từ Odom Quaternion nếu chưa có IMU
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw_odom = math.atan2(siny_cosp, cosy_cosp)
            deg = math.degrees(yaw_odom)
            if deg < 0: deg += 360
            heading_cdeg = int(deg * 100)

        self.mav.mav.global_position_int_send(
            self.get_boot_time(),
            int(current_lat * 1e7), int(current_lon * 1e7),
            int(10 * 1000), int(10 * 1000),
            int(msg.twist.twist.linear.x * 100),
            int(msg.twist.twist.linear.y * 100),
            0, heading_cdeg
        )
        # Lưu ý: Không gửi attitude_send ở đây nữa vì imu_callback đã lo rồi




    def read_mavlink_loop(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
            if not msg: break
            
            msg_type = msg.get_type()

            # --- IDENTIFICATION ---
            if msg_type == 'COMMAND_LONG' and msg.command == 520:
                self.mav.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                self.send_autopilot_version() # Trả lời ngay khi được hỏi
                continue
            
            if msg_type == 'COMMAND_LONG' and msg.command == 519:
                 self.mav.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                 self.mav.mav.protocol_version_send(200, 200, 200, bytearray(), bytearray())
                 continue

            # --- ARMING ---
            if msg_type == 'COMMAND_LONG' and msg.command == 400:
                self.is_armed = int(msg.param1) == 1
                arm_msg = Bool()
                arm_msg.data = self.is_armed
                self.arm_pub.publish(arm_msg)
                self.get_logger().info(f"ARM COMMAND: {self.is_armed}")
                self.mav.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                self.send_heartbeat() # Cập nhật trạng thái ngay lập tức
                continue

            # --- MISSION ---
            if msg_type == 'MISSION_COUNT':
                self.mission_count = msg.count
                self.current_wp_seq = 0
                self.mav.mav.mission_request_int_send(self.sys_id, self.comp_id, 0)
                self.current_path = Path()
                self.current_path.header.frame_id = "map"

            elif msg_type == 'MISSION_ITEM_INT':
                if msg.seq == self.current_wp_seq:
                    self.process_waypoint(msg)
                    self.current_wp_seq += 1
                    if self.current_wp_seq < self.mission_count:
                        self.mav.mav.mission_request_int_send(self.sys_id, self.comp_id, self.current_wp_seq)
                    else:
                        self.mav.mav.mission_ack_send(self.sys_id, self.comp_id, mavutil.mavlink.MAV_MISSION_ACCEPTED)
                        self.path_pub.publish(self.current_path)

            elif msg_type == 'MISSION_CLEAR_ALL':
                self.current_path = Path()
                self.current_path.header.frame_id = "map"
                self.path_pub.publish(self.current_path)
                self.mav.mav.mission_ack_send(self.sys_id, self.comp_id, mavutil.mavlink.MAV_MISSION_ACCEPTED)

            # --- OTHERS ---
            elif msg_type == 'COMMAND_INT':
                 self.mav.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)

            elif msg_type == 'PARAM_REQUEST_LIST':
                self.send_all_params()

    def process_waypoint(self, msg):
        if self.origin_lat is None: return
        if msg.command != 16: return
        wp_lat = msg.x / 1e7
        wp_lon = msg.y / 1e7
        R = 6378137.0 
        dLat = math.radians(wp_lat - self.origin_lat)
        dLon = math.radians(wp_lon - self.origin_lon)
        lat0_rad = math.radians(self.origin_lat)
        x = dLon * R * math.cos(lat0_rad)
        y = dLat * R
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        self.current_path.header.frame_id = "map"
        self.current_path.header.stamp = self.get_clock().now().to_msg()
        self.current_path.poses.append(pose)

    def send_all_params(self):
        for i, key in enumerate(self.param_keys):
            self.mav.mav.param_value_send(
                key.encode('utf-8'), self.param_dict[key], 
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, len(self.param_keys), i)
            time.sleep(0.005)

def main(args=None):
    rclpy.init(args=args)
    node = QGCBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()