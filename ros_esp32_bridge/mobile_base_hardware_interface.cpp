#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>
#include "wheel.h"

namespace mobile_base_hardware
{

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Initing...");

    time_ = std::chrono::system_clock::now();

    port_name_ = info_.hardware_parameters.at("port");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    enc_counts_per_revolution_ = std::stoi(info_.hardware_parameters.at("enc_counts_per_revolution"));
    loop_rate = std::stof(info_.hardware_parameters.at("loop_rate"));
    
    left_wheel_.setup(info_.joints[0].name, enc_counts_per_revolution_);
    right_wheel_.setup(info_.joints[1].name, enc_counts_per_revolution_);

    RCLCPP_INFO(logger_, "Finished Init");


    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
   // RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Configuring Serial Port: %s", port_name_.c_str());

    try {
        serial_port_.Open(port_name_);
        serial_port_.SetBaudRate((LibSerial::BaudRate)baud_rate_);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    } catch (const LibSerial::OpenFailed &) {
        RCLCPP_ERROR(logger_, "Failed to open serial port: %s", port_name_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)

{
    (void)previous_state;
    RCLCPP_INFO(logger_, "Activating...");
    serial_port_.Write("\r");
    std::stringstream ss;
    float k_p = 30.0;
    float k_d = 20.0;
    float k_i = 0.0;
    float k_o = 100.0;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    serial_port_.Write(ss.str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(logger_, "Deactivating...");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;    
    
    if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

    std::string response;
    serial_port_.Write("e\r");
    serial_port_.ReadLine(response);

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    left_wheel_.enc = std::atoi(token_1.c_str());
    right_wheel_.enc = std::atoi(token_2.c_str());
    try {
         
        
        RCLCPP_INFO(logger_, "ESP32 SAYS: '%s'", response.c_str());

    } catch (...) {
        return hardware_interface::return_type::OK;
    }
      
    double pos_prev = left_wheel_.pos;
    left_wheel_.pos = left_wheel_.calcEncAngle();
    left_wheel_.vel = (left_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = right_wheel_.pos;
    right_wheel_.pos = right_wheel_.calcEncAngle();
    right_wheel_.vel = (right_wheel_.pos - pos_prev) / deltaSeconds;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), 
        "L_CMD: %lf, R_CMD: %lf, L_POS: %lf, R_POS: %lf",
        left_wheel_.cmd, 
        right_wheel_.cmd, 
        left_wheel_.pos, 
        right_wheel_.pos);

    std::stringstream ss;
    int val_1 ,val_2;
    val_1=left_wheel_.cmd / left_wheel_.rads_per_count / loop_rate;
    val_2=right_wheel_.cmd / right_wheel_.rads_per_count / loop_rate;
    ss << "m " << val_1 << " " << val_2 << "\r";
    serial_port_.Write(ss.str());

    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mobile_base_hardware::MobileBaseHardwareInterface,
    hardware_interface::SystemInterface)