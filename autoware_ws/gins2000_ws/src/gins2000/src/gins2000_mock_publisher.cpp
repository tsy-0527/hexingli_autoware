#include <rclcpp/rclcpp.hpp>
#include <gins2000/msg/gins2000.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <chrono>

#define G       9.87
#define deg2rad (M_PI / 180.0)

class Gins2000MockPublisher : public rclcpp::Node
{
public:
    Gins2000MockPublisher() : Node("gins2000_mock_publisher"), frame_count_(0)
    {
        topic_     = this->declare_parameter<std::string>("gins2000_topic", "gins2000");
        frame_id_  = this->declare_parameter<std::string>("gins2000_frame_id", "base_link");
        freq_hz_   = this->declare_parameter<double>("publish_freq_hz", 1.0);

        pub_ = this->create_publisher<gins2000::msg::Gins2000>(topic_, 10);

        auto period_ms = std::chrono::duration<double, std::milli>(1000.0 / freq_hz_);
        timer_ = this->create_wall_timer(period_ms,
                     std::bind(&Gins2000MockPublisher::publishMock, this));

        RCLCPP_INFO(this->get_logger(),
                    "Mock publisher started on topic [%s] @ %.1f Hz",
                    topic_.c_str(), freq_hz_);
    }

private:
    void publishMock()
    {
        double t = frame_count_ / freq_hz_;   // 模拟时间（秒）

        gins2000::msg::Gins2000 msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = frame_id_;

        // ── 帧计数 ──────────────────────────────────────────────
        msg.frame_count = frame_count_;

        // ── 姿态（欧拉角，单位：度，与原始驱动保持一致） ───────
        //   pitch 在 ±30° 内正弦摆动
        //   roll  在 ±10° 内正弦摆动
        //   yaw   缓慢旋转 0→360°
        msg.attitude.x = 10.0  * std::sin(2.0 * M_PI * 0.2 * t);   // roll
        msg.attitude.y = 30.0  * std::sin(2.0 * M_PI * 0.1 * t);   // pitch
        msg.attitude.z = std::fmod(45.0 * t, 360.0);                // yaw

        // ── 角速度（rad/s） ─────────────────────────────────────
        //   对应上面姿态的导数（近似正弦波）
        msg.angular_velocity.x = (10.0  * 2*M_PI*0.2) * std::cos(2*M_PI*0.2*t) * deg2rad;
        msg.angular_velocity.y = (30.0  * 2*M_PI*0.1) * std::cos(2*M_PI*0.1*t) * deg2rad;
        msg.angular_velocity.z = 45.0 * deg2rad;   // 匀速偏航

        // ── 线加速度（m/s²） ────────────────────────────────────
        //   Z 轴为重力基准，加小幅度 XY 振动
        msg.linear_acceleration.x = 0.1 * std::sin(2*M_PI*1.0*t) * G;
        msg.linear_acceleration.y = 0.1 * std::cos(2*M_PI*1.0*t) * G;
        msg.linear_acceleration.z = 1.0 * G;    // ~9.87 m/s²

        // ── 位置（WGS84，模拟在某固定点附近微小漂移） ──────────
        msg.latitude  = 31.9339385 + 1e-5 * std::sin(2*M_PI*0.05*t);
        msg.longitude = 118.6332007 + 1e-5 * std::cos(2*M_PI*0.05*t);
        msg.altitude  = 10.0 + 0.5 * std::sin(2*M_PI*0.02*t);

        // ── 速度（ENU，m/s） ────────────────────────────────────
        msg.velocity.x = 1.0 * std::cos(2*M_PI*0.05*t);   // 东向
        msg.velocity.y = 1.0 * std::sin(2*M_PI*0.05*t);   // 北向
        msg.velocity.z = 0.05 * std::sin(2*M_PI*0.02*t);  // 天向

        // ── INS 状态 ────────────────────────────────────────────
        //   前 5 秒状态为 0（未对准），之后置为 4（组合导航）
        msg.ins_status = (t < 5.0) ? 0 : 4;

        // ── GPS 时间 ─────────────────────────────────────────────
        //   GPS Week 2320 起点，按频率累加毫秒
        msg.week        = 2320;
        msg.gps_millisec = static_cast<uint32_t>(t * 1000.0) % (7 * 24 * 3600 * 1000);

        // ── 温度（°C） ──────────────────────────────────────────
        msg.temperature = 25.0f + 0.5f * static_cast<float>(std::sin(2*M_PI*0.01*t));

        pub_->publish(msg);
        frame_count_++;
    }

    rclcpp::Publisher<gins2000::msg::Gins2000>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string topic_;
    std::string frame_id_;
    double      freq_hz_;
    uint32_t    frame_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gins2000MockPublisher>());
    rclcpp::shutdown();
    return 0;
}