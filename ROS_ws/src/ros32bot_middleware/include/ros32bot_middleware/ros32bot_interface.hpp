#ifndef ROS32BOT_MIDDLEWARE_INTERFACE_HPP
#define ROS32BOT_MIDDLEWARE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

#include <cstddef>
#include <string>
#include <vector>

namespace ros32bot_middleware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Ros32BotInterface : public hardware_interface::SystemInterface
{

public:
    Ros32BotInterface();
    virtual ~Ros32BotInterface();

    virtual CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & prev_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev_state) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    LibSerial::SerialPort serial_port_;
    std::string port_;
    double wheel_radius_;
    std::size_t left_command_index_;
    std::size_t right_command_index_;
    std::vector<double> velocity_commands_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
};
}


#endif
