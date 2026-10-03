#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

#include "ros32bot_middleware/package_bytes.hpp"

namespace
{
// Wire format is centirad/s; matches WHEEL_VEL_CRAD_PER_RAD in uart_comms.h.
constexpr double CRAD_PER_RAD = 100.0;
}

namespace ros32bot_middleware
{

// Mixes cmd_vel into per-wheel rad/s and streams the frames to the STM32.
class UartBridge : public rclcpp::Node
{
public:
    UartBridge()
    : Node("uart_bridge")
    {
        port_ = declare_parameter<std::string>("port", "/dev/ttyAMA0");
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
        wheel_separation_ = declare_parameter<double>("wheel_separation", 0.17);
        cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
        const double publish_rate = declare_parameter<double>("publish_rate", 50.0);
        const std::string topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

        serial_port_.Open(port_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            topic, 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { onCmdVel(msg); });

        last_command_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            [this]() { onTimer(); });

        RCLCPP_INFO(get_logger(), "uart_bridge writing to %s, listening on %s",
                    port_.c_str(), topic.c_str());
    }

    ~UartBridge() override
    {
        if (!serial_port_.IsOpen())
        {
            return;
        }

        // Leave the STM32 holding an explicit zero rather than the last command.
        sendWheelSpeeds(0.0, 0.0);
        serial_port_.Close();
    }

private:
    // Clamped so a wild command cannot wrap around into a large opposite value.
    static int16_t toCentirad(double rad_s)
    {
        const double crad_s = rad_s * CRAD_PER_RAD;

        if (crad_s > 32767.0)
        {
            return 32767;
        }
        if (crad_s < -32768.0)
        {
            return -32768;
        }

        return static_cast<int16_t>(crad_s);
    }

    void onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        linear_ = msg->twist.linear.x;
        angular_ = msg->twist.angular.z;
        last_command_time_ = now();
    }

    void onTimer()
    {
        // Watchdog: stop rather than repeat a command the PC stopped sending.
        const bool fresh = (now() - last_command_time_).seconds() < cmd_vel_timeout_;

        if (!fresh)
        {
            sendWheelSpeeds(0.0, 0.0);
            return;
        }

        sendWheelSpeeds(linear_, angular_);
    }

    void sendWheelSpeeds(double linear, double angular)
    {
        const double half_separation = wheel_separation_ / 2.0;
        const double left_rad_s = (linear - angular * half_separation) / wheel_radius_;
        const double right_rad_s = (linear + angular * half_separation) / wheel_radius_;

        const Bytes frame = packageWheelVelocities(toCentirad(left_rad_s), toCentirad(right_rad_s));

        try
        {
            serial_port_.Write(frame);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "write failed: %s", e.what());
        }
    }

    LibSerial::SerialPort serial_port_;
    std::string port_;
    double wheel_radius_;
    double wheel_separation_;
    double cmd_vel_timeout_;
    double linear_ = 0.0;
    double angular_ = 0.0;
    rclcpp::Time last_command_time_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        // The constructor opens the serial port, so a failure lands here instead of aborting.
        rclcpp::spin(std::make_shared<ros32bot_middleware::UartBridge>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("uart_bridge"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
