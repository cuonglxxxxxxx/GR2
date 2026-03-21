#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include "my_robot_hardware/wheel.h"

namespace mobile_base_hardware
{

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(logger_, "Starting on_init...");
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    time_ = std::chrono::system_clock::now();

    try {
        port_name_ = info.hardware_parameters.at("port");
        baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
        enc_counts_per_revolution_ = std::stoi(info.hardware_parameters.at("enc_counts_per_revolution"));
        // loop_rate_ is not used anymore, but we still parse it if present
        if (info.hardware_parameters.count("loop_rate") > 0) {
            loop_rate_ = std::stof(info.hardware_parameters.at("loop_rate"));
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Error parsing hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (info.joints.size() < 2) {
        RCLCPP_ERROR(logger_, "Expected 2 joints, but got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    right_wheel_.setup(info.joints[0].name, enc_counts_per_revolution_);
    left_wheel_.setup(info.joints[1].name, enc_counts_per_revolution_);

    RCLCPP_INFO(logger_, "on_init successful");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(logger_, "Exporting state interfaces...");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(logger_, "Exporting command interfaces...");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));
    return command_interfaces;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Configuring Serial Port: %s at %d baud", port_name_.c_str(), baud_rate_);

    if (serial_port_.IsOpen()) {
        serial_port_.Close();
    }

    try {
        serial_port_.Open(port_name_);
        
        LibSerial::BaudRate baud;
        switch (baud_rate_) {
            case 9600: baud = LibSerial::BaudRate::BAUD_9600; break;
            case 57600: baud = LibSerial::BaudRate::BAUD_57600; break;
            case 115200: baud = LibSerial::BaudRate::BAUD_115200; break;
            default: baud = LibSerial::BaudRate::BAUD_57600; break;
        }

        serial_port_.SetBaudRate(baud);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        RCLCPP_INFO(logger_, "Serial port configured. Waiting for ESP32...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Failed to open serial port: %s. Error: %s", port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)

{
    (void)previous_state;
    RCLCPP_INFO(logger_, "Activating...");
    
    // Reset time_ here to ensure deltaSeconds in first read() is correct
    time_ = std::chrono::system_clock::now();

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
    
    if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

    try {
        std::string response;
        serial_port_.Write("e\r");
        
        try {
            serial_port_.ReadLine(response, 10);
        } catch (const LibSerial::ReadTimeout &) {
            return hardware_interface::return_type::OK;
        }

        if (response.empty()) return hardware_interface::return_type::OK;

        // Xóa ký tự xuống dòng
        response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
        response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());

        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        if (del_pos != std::string::npos && del_pos > 0 && del_pos < response.length() - 1) {
            std::string token_1 = response.substr(0, del_pos);
            std::string token_2 = response.substr(del_pos + delimiter.length());

            if (!token_1.empty() && !token_2.empty()) {
                try {
                    left_wheel_.enc = std::stoi(token_1);
                    right_wheel_.enc = std::stoi(token_2);
                } catch (...) {
                    // RCLCPP_WARN(logger_, "Failed to parse encoder values from: %s", response.c_str());
                }
            }
        }

    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Error during read serial: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
      
    // Cập nhật thời gian sau khi đọc xong để deltaSeconds chính xác cho lần tới
    time_ = new_time;

    double pos_prev_l = left_wheel_.pos;
    left_wheel_.pos = left_wheel_.calcEncAngle();
    
    double pos_prev_r = right_wheel_.pos;
    right_wheel_.pos = right_wheel_.calcEncAngle();

    // Chỉ tính vận tốc nếu thời gian trôi qua đủ lớn (> 1ms)
    if (deltaSeconds > 0.001) {
        left_wheel_.vel = (left_wheel_.pos - pos_prev_l) / deltaSeconds;
        right_wheel_.vel = (right_wheel_.pos - pos_prev_r) / deltaSeconds;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

    try {
        std::stringstream ss;
        int val_1, val_2;
        
        // Tránh chia cho 0 hoặc giá trị quá nhỏ của loop_rate_
        float rate = (loop_rate_ > 0) ? loop_rate_ : 30.0;
        
        val_1 = static_cast<int>(left_wheel_.cmd / left_wheel_.rads_per_count / rate);
        val_2 = static_cast<int>(right_wheel_.cmd / right_wheel_.rads_per_count / rate);
        
        ss << "m " << val_1 << " " << val_2 << "\r";
        serial_port_.Write(ss.str());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Error during write: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mobile_base_hardware::MobileBaseHardwareInterface,
    hardware_interface::SystemInterface)