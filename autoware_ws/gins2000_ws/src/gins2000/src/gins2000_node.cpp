#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <gins2000/msg/gins2000.hpp>
#include <string>
#include <cmath>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdint>

#define G  9.87
#define deg2rad M_PI/180.0
#define rad2deg 180/M_PI

struct GINS_DATA
{
    float pitch;
    float roll;
    float yaw;
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;
    float Mx;
    float My;
    float Mz;
    double Lat;
    double Lon;
    double hig;
    float Ve;
    float Vn;
    float Vu;
    uint8_t status_39;
    uint32_t count;
    uint8_t status_44;
    uint8_t status_45;
    float Lat_std;
    float Lon_std;
    float Hig_std;
    float Ve_std;
    float Vn_std;
    float Vu_std;
    float yaw_std;
    float pitch_std;
    float roll_std;
    float Data;
    uint16_t week;
    uint32_t gps_mis;
    uint8_t type;
};

std::string stringToHex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    std::size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (std::size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}

std::vector<uint8_t> stringToUint8(const std::string& str)
{
    std::vector<uint8_t> result;
    result.assign(str.begin(), str.end());
    return result;
}

int16_t readInt16LE(const uint8_t* data)
{
    const uint16_t raw =
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
    return static_cast<int16_t>(raw);
}

int32_t readInt32LE(const uint8_t* data)
{
    const uint32_t raw =
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24);
    return static_cast<int32_t>(raw);
}

uint16_t readUint16LE(const uint8_t* data)
{
    return
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readUint32LE(const uint8_t* data)
{
    return
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24);
}

class Gins2000Node : public rclcpp::Node
{
public:
    Gins2000Node() : Node("gins2000_node")
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        frame_id_ = this->declare_parameter<std::string>("gins2000_frame_id", "base_link");
        cfg_baud_rate_ = this->declare_parameter<int>("gins2000_baudrate", 921600);
        topic_ = this->declare_parameter<std::string>("gins2000_topic", "gins2000");

        imu_pub_ = this->create_publisher<gins2000::msg::Gins2000>(topic_, 10);

        try
        {
            ser_.setPort(port_);
            ser_.setBaudrate(cfg_baud_rate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port !!!");
            rclcpp::shutdown();
            return;
        }

        if (ser_.isOpen())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port initialized ok");
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Serial Port initialized failed");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Gins2000 start work...");

        // ✅ 用 Timer 替代子线程，1ms 轮询一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&Gins2000Node::serialLoop, this)
        );
    }

    ~Gins2000Node()
    {
        if (ser_.isOpen())
        {
            ser_.close();
        }
    }

private:
    void serialLoop()
    {
        constexpr std::size_t size = 74;
        GINS_DATA ginsData;
        int16_t temp1 = 0;
        int32_t temp4 = 0;
        uint32_t temp3;
        uint16_t temp5 = 0;

        try
        {
            if (ser_.isOpen())
            {
                if (ser_.available())
                {
                    std::string input_0 = ser_.read(size);
                    std::string input_hex = stringToHex(input_0);
                    // printf("%u\n",input_hex);

                   // RCLCPP_INFO_STREAM(this->get_logger(), "RAW: " << input_hex); // 调试用，确认后可删

                    std::string input;
                    std::string::size_type position;
                    position = input_hex.find("BDDB0A");

                    if (position != std::string::npos)
                    {
                        if (position != 0)
                        {
                            std::string input_1 = ser_.read((position / 2));
                            input = input_0 + input_1;
                        }
                        else
                        {
                            input = input_0;
                        }

                        const std::size_t frame_offset = position / 2;
                        if (input.size() < frame_offset + size)
                        {
                            RCLCPP_WARN_STREAM(
                                this->get_logger(),
                                "Incomplete frame: got " << (input.size() - frame_offset)
                                << " bytes, expected " << static_cast<int>(size));
                            return;
                        }

                        const auto* input_uint8 = reinterpret_cast<const uint8_t*>(
                            input.data() + frame_offset);
                        uint8_t sum = 0;
                        for (std::size_t i = 0; i < size - 1; i++)
                        {
                            sum ^= input_uint8[i];
                        }

			if (sum != static_cast<uint8_t>(input_uint8[73]))
			{
			    RCLCPP_ERROR_STREAM(this->get_logger(), 
				"校验失败: 计算值=" << (int)sum << " 期望值=" << (int)(uint8_t)input_uint8[73]);
			}

                        if (sum != static_cast<uint8_t>(input_uint8[73]))
                        {
                    
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Frame verification failed!!!");
                        }
                        else
                        {
                            // 姿态角
                            temp1 = readInt16LE(input_uint8 + 3);
                            ginsData.pitch = temp1 * 360 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 5);
                            ginsData.roll = temp1 * 360 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 7);
                            ginsData.yaw = temp1 * 360 / 32768.0;
                    

                            // 陀螺
                            temp1 = readInt16LE(input_uint8 + 9);
                            ginsData.Gx = temp1 * 450 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 11);
                            ginsData.Gy = temp1 * 450 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 13);
                            ginsData.Gz = temp1 * 450 / 32768.0;

                            // 加计
                            temp1 = readInt16LE(input_uint8 + 15);
                            ginsData.Ax = temp1 * 16 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 17);
                            ginsData.Ay = temp1 * 16 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 19);
                            ginsData.Az = temp1 * 16 / 32768.0;

                            // 位置
                            temp4 = readInt32LE(input_uint8 + 21);
                            ginsData.Lat = temp4 * 1e-7;
                            temp4 = readInt32LE(input_uint8 + 25);
                            ginsData.Lon = temp4 * 1e-7;
                            temp4 = readInt32LE(input_uint8 + 29);
                            ginsData.hig = temp4 * 1e-3;
                            // printf("%d \t%d \t%d \t%d \t\n",input_uint8[21],input_uint8[22],input_uint8[23],input_uint8[24]);
                           
                            // 速度
                            temp1 = readInt16LE(input_uint8 + 33);
                            ginsData.Ve = temp1 * 100 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 35);
                            ginsData.Vn = temp1 * 100 / 32768.0;
                            temp1 = readInt16LE(input_uint8 + 37);
                            ginsData.Vu = temp1 * 100 / 32768.0;

                            // 组合状态和帧计数
                            ginsData.status_39 = input_uint8[39];
                            temp3 = readUint32LE(input_uint8 + 40);
                            ginsData.count = temp3;

                            // 卫星接收机定位和定向状态
                            ginsData.status_44 = input_uint8[44];

                            // IMU 和卫星接收机状态
                            ginsData.status_45 = input_uint8[45];

                            // 纬度、经度、高度标准差
                            temp5 = readUint16LE(input_uint8 + 46);
                            ginsData.Lat_std = temp5 * 600 / 65535.0;
                            temp5 = readUint16LE(input_uint8 + 48);
                            ginsData.Lon_std = temp5 * 600 / 65535.0;
                            temp5 = readUint16LE(input_uint8 + 50);
                            ginsData.Hig_std = temp5 * 2000 / 65535.0;

                            // 东、北、天向速度标准差
                            temp5 = readUint16LE(input_uint8 + 52);
                            ginsData.Ve_std = temp5 * 40 / 65535.0;
                            temp5 = readUint16LE(input_uint8 + 54);
                            ginsData.Vn_std = temp5 * 40 / 65535.0;
                            temp5 = readUint16LE(input_uint8 + 56);
                            ginsData.Vu_std = temp5 * 75 / 65535.0;

                            // yaw、pitch、roll 标准差
                            temp5 = readUint16LE(input_uint8 + 58);
                            ginsData.yaw_std = temp5 * 20 / 65535.0;
                            temp5 = readUint16LE(input_uint8 + 60);
                            ginsData.pitch_std = temp5 * 20 / 65535.0;
                            temp5 = readUint16LE(input_uint8 + 62);
                            ginsData.roll_std = temp5 * 20 / 65535.0;

                            // Data（Type 为 22 时为温度数据）
                            temp1 = readInt16LE(input_uint8 + 64);
                            ginsData.Data = temp1 * 200 / 32768.0;

                            // 周
                            ginsData.week = readUint16LE(input_uint8 + 66);

                            // 周内秒/ms
                            temp3 = readUint32LE(input_uint8 + 68);
                            ginsData.gps_mis = temp3;

                            // Type
                            ginsData.type = input_uint8[72];

                            // 填充并发布消息
                            auto msg = gins2000::msg::Gins2000();
                            msg.attitude.x = ginsData.roll;
                            msg.attitude.y = ginsData.pitch;
                            msg.attitude.z = ginsData.yaw;
                            //msg.attitude.z = msg.attitude.z+90;
                            msg.angular_velocity.x = ginsData.Gx * deg2rad;
                            msg.angular_velocity.y = ginsData.Gy * deg2rad;
                            msg.angular_velocity.z = ginsData.Gz * deg2rad;
                            msg.linear_acceleration.x = ginsData.Ax * G;
                            msg.linear_acceleration.y = ginsData.Ay * G;
                            msg.linear_acceleration.z = ginsData.Az * G;
                            msg.latitude = ginsData.Lat;
                            msg.longitude = ginsData.Lon;
                            msg.altitude = ginsData.hig;
                            msg.velocity.x = ginsData.Ve;
                            msg.velocity.y = ginsData.Vn;
                            msg.velocity.z = ginsData.Vu;
                            msg.ins_status = ginsData.status_39;
                            msg.gnss_fix_status = ginsData.status_44;
                            msg.imu_gnss_comm_status = ginsData.status_45;
                            msg.latitude_std = ginsData.Lat_std;
                            msg.longitude_std = ginsData.Lon_std;
                            msg.altitude_std = ginsData.Hig_std;
                            msg.ve_std = ginsData.Ve_std;
                            msg.vn_std = ginsData.Vn_std;
                            msg.vu_std = ginsData.Vu_std;
                            msg.yaw_std = ginsData.yaw_std;
                            msg.pitch_std = ginsData.pitch_std;
                            msg.roll_std = ginsData.roll_std;
                            msg.week = ginsData.week;
                            msg.frame_count = ginsData.count;
                            msg.gps_millisec = ginsData.gps_mis;
                            msg.temperature = ginsData.Data;
                            msg.header.stamp = this->now();
                            msg.header.frame_id = frame_id_;

                            imu_pub_->publish(msg);
                        }
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "not find bddb0a !");
                    }
                }
            }
            else
            {
                try
                {
                    ser_.setPort(port_);
                    ser_.setBaudrate(cfg_baud_rate_);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser_.setTimeout(to);
                    ser_.open();
                }
                catch (serial::IOException& e)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open serial port, retry in 5s.");
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
            }
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Error reading from serial port, closing.");
            ser_.close();
        }
    }

    serial::Serial ser_;
    rclcpp::TimerBase::SharedPtr timer_;  // ✅ 用 Timer 替代 thread

    std::string port_;
    std::string frame_id_;
    int cfg_baud_rate_;
    std::string topic_;

    rclcpp::Publisher<gins2000::msg::Gins2000>::SharedPtr imu_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Gins2000Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
