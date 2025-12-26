import time
import serial

def calculate_checksum(sentence):
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return "{:02X}".format(checksum)

def convert_to_nmea(degrees):
    # Chuyển đổi decimal degrees sang format NMEA (DDDMM.MMMM)
    d = int(degrees)
    m = (degrees - d) * 60
    return d, m

def main():
    # Mở cổng đầu vào
    try:
        ser = serial.Serial('/dev/ttbot_gps_in', 9600, timeout=1)
    except serial.SerialException:
        print("Loi: Khong tim thay cong /dev/ttbot_gps_in. Ban da chay lenh socat chua?")
        return

    # Tọa độ Nhà thờ Đức Bà
    lat = 10.7798
    lon = 106.6990
    altitude = 15.5  # Mét
    
    print("Dang ban full data (GGA + RMC) vao /dev/ttbot_gps ...")

    while True:
        # Giả lập di chuyển nhẹ
        lat += 0.000005
        lon += 0.000005
        
        # Lấy giờ UTC hiện tại
        now = time.gmtime()
        time_str = time.strftime("%H%M%S", now) + ".00"
        date_str = time.strftime("%d%m%y", now)

        # Xử lý tọa độ
        lat_d, lat_m = convert_to_nmea(lat)
        lon_d, lon_m = convert_to_nmea(lon)
        
        lat_str = "{:02d}{:07.4f}".format(lat_d, lat_m)
        lon_str = "{:03d}{:07.4f}".format(lon_d, lon_m)

        # 1. Tạo câu lệnh GPGGA (Global Positioning System Fix Data)
        # Cấu trúc: Time, Lat, N, Lon, E, Quality(1=GPS fix), NumSats(8), HDOP(0.9), Alt, M, Sep, M,,*CS
        base_gga = "GPGGA,{},{},N,{},E,1,08,0.9,{},M,0.0,M,,".format(time_str, lat_str, lon_str, altitude)
        nmea_gga = "${}*{}\r\n".format(base_gga, calculate_checksum(base_gga))

        # 2. Tạo câu lệnh GNRMC (Recommended Minimum)
        # Cấu trúc: Time, Status(A=OK), Lat, N, Lon, E, Speed, Angle, Date,,,Mode*CS
        base_rmc = "GNRMC,{},A,{},N,{},E,0.1,0.0,{},,,A".format(time_str, lat_str, lon_str, date_str)
        nmea_rmc = "${}*{}\r\n".format(base_rmc, calculate_checksum(base_rmc))

        # Gửi cả 2 câu lệnh
        ser.write(nmea_gga.encode('ascii'))
        ser.write(nmea_rmc.encode('ascii'))
        
        print(f"Sent: GGA & RMC at {time_str}")
        
        # NMEA chuẩn thường là 1Hz hoặc 5Hz. Để 5Hz (0.2s) cho mượt.
        time.sleep(0.2)

if __name__ == "__main__":
    main()