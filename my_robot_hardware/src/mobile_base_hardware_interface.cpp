#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

namespace mobile_base_hardware
{

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.count("dynamixel_port")) {
        port_name_ = info_.hardware_parameters.at("dynamixel_port"); 
    } else {
        port_name_ = "/dev/ttyACM0";
    }
    
    baud_rate_ = 57600;

    left_wheel_.name = info_.joints[0].name;
    right_wheel_.name = info_.joints[1].name;

    left_wheel_.position = 0.0;
    left_wheel_.velocity = 0.0;
    left_wheel_.command = 0.0;

    right_wheel_.position = 0.0;
    right_wheel_.velocity = 0.0;
    right_wheel_.command = 0.0;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command));

    return command_interfaces;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Configuring Serial Port: %s", port_name_.c_str());

    try {
        serial_port_.Open(port_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    } catch (const LibSerial::OpenFailed &) {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"), "Failed to open serial port: %s. Check permissions!", port_name_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Activating...");
    
    for (const auto & joint : info_.joints) {
        RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "JOINT NAME: %s", joint.name.c_str());
        for (const auto & interface : joint.command_interfaces) {
            RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "  COMMAND INTERFACE: %s", interface.name.c_str());
        }
        for (const auto & interface : joint.state_interfaces) {
            RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "  STATE INTERFACE: %s", interface.name.c_str());
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Deactivating...");
    if (serial_port_.IsOpen()) {
        serial_port_.Write("o 0 0\r");
        serial_port_.Close();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

    serial_port_.Write("e\r");

    std::string response;
    try {
        serial_port_.ReadLine(response, '\n', 20); 
        
        // --- DEBUG: IN RA CHUỖI THÔ NHẬN ĐƯỢC ---
        RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "ESP32 SAYS: '%s'", response.c_str());
        // ----------------------------------------

    } catch (...) {
        return hardware_interface::return_type::OK;
    }

    std::stringstream ss(response);
    long left_enc_val, right_enc_val;
    
    if (ss >> left_enc_val >> right_enc_val) {
        double ticks_per_rev = 2464.0; 
        
        left_wheel_.position = (left_enc_val / ticks_per_rev) * 2.0 * M_PI;
        right_wheel_.position = (right_enc_val / ticks_per_rev) * 2.0 * M_PI;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), 
        "L_CMD: %lf, R_CMD: %lf, L_POS: %lf, R_POS: %lf",
        left_wheel_.command, right_wheel_.command, left_wheel_.position, right_wheel_.position);

    double max_speed_rad = 18.43; 
    
    int left_pwm = (int)((left_wheel_.command / max_speed_rad) * 255.0);
    int right_pwm = (int)((right_wheel_.command / max_speed_rad) * 255.0);

    if (left_pwm > 255) left_pwm = 255;
    if (left_pwm < -255) left_pwm = -255;
    if (right_pwm > 255) right_pwm = 255;
    if (right_pwm < -255) right_pwm = -255;

    std::stringstream cmd_ss;
    cmd_ss << "o " << left_pwm << " " << right_pwm << "\r";
    serial_port_.Write(cmd_ss.str());

    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mobile_base_hardware::MobileBaseHardwareInterface,
    hardware_interface::SystemInterface)