# GnssIns.msg
# GNSS/INS组合导航输出消息

# 标准头信息
std_msgs/Header header

#帧计数
uint32 frame_count

# 姿态信息 (欧拉角)
geometry_msgs/Vector3 attitude   # 滚转、俯仰、偏航 [度]

# 角速度 [弧度/秒]
geometry_msgs/Vector3 angular_velocity

# 加速度 [米/秒²]
geometry_msgs/Vector3 linear_acceleration


# 位置信息 (WGS84坐标系)
float64 latitude          # 纬度 [度]
float64 longitude         # 经度 [度]
float64 altitude          # 海拔高度 [米]

# 速度信息 (ENU坐标系)
geometry_msgs/Vector3 velocity   # 东向、北向、天向速度 [米/秒]

#状态
uint8 ins_status          #组合状态和周内秒有效标志
uint8 gnss_fix_status       #卫星接收机定位和定向状态
uint8 imu_gnss_comm_status      #IMU和卫星接收机状态

#位置、速度、姿态标准差
float32 latitude_std          #纬度标准差[m]
float32 longitude_std      #经度标准差[m]
float32 altitude_std         #高度标准差[m]
float32 ve_std                 #东速标准差[m/s]
float32 vn_std                 #北速标准差[m/s]
float32 vu_std                 #天速标准差[m/s]
float32 yaw_std              #航向角标准差[deg]
float32 pitch_std             #俯仰角标准差[deg]
float32 roll_std                #横滚角标准差[deg]

#周
uint16 week
#周内秒
uint32 gps_millisec

#温度
float32 temperature
