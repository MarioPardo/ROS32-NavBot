#include "ros32bot_middleware/ros32bot_interface.hpp"
#include "ros32bot_middleware/package_bytes.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>

namespace ros32bot_middleware
{

Ros32BotInterface::Ros32BotInterface()
: wheel_radius_(0.033),
  left_command_index_(0),
  right_command_index_(0)
{
}

Ros32BotInterface::~Ros32BotInterface()
{
    if(serial_port_.IsOpen())
    {
        try
        {
            serial_port_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Error closing port: " << port_);
        }
    }
}

CallbackReturn Ros32BotInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(params);

    if(result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("Ros32BotInterface"), "No serial port provided.. ABORTING!");
        return CallbackReturn::FAILURE;
    }

    try
    {
        wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
    }
    catch(const std::exception &)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("Ros32BotInterface"), "No wheel_radius provided, using " << wheel_radius_);
    }

    velocity_commands_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);

    for(std::size_t i = 0; i < info_.joints.size(); i++)
    {
        if(info_.joints[i].name.find("left") != std::string::npos)
            left_command_index_ = i;
        else if(info_.joints[i].name.find("right") != std::string::npos)
            right_command_index_ = i;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn Ros32BotInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Starting robot hardware...");

    try
    {
        serial_port_.Open(port_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Error when interacting with port: " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Hardware started! Sending commands on port: " << port_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn Ros32BotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Stopping robot hardware...");

    try
    {
        serial_port_.Close();
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Error when closing port: " << port_);
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Ros32BotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(std::size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &velocity_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Ros32BotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(std::size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &velocity_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type Ros32BotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ros32BotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // Joint velocities are rad/s; the wire carries centirad/s.
    const int16_t left_crad_s = static_cast<int16_t>(velocity_commands_.at(left_command_index_) * 100.0);
    const int16_t right_crad_s = static_cast<int16_t>(velocity_commands_.at(right_command_index_) * 100.0);

    const Bytes frame = packageWheelVelocities(left_crad_s, right_crad_s);

    try
    {
        serial_port_.Write(frame);
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Ros32BotInterface"), "Error when sending wheel velocities on port: " << port_);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(ros32bot_middleware::Ros32BotInterface, hardware_interface::SystemInterface)
