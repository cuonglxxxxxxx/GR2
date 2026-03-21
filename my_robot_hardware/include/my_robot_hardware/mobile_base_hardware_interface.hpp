#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include "my_robot_hardware/wheel.h"

namespace mobile_base_hardware {

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
    MobileBaseHardwareInterface() 
        : logger_(rclcpp::get_logger("MobileBaseHardwareInterface")),
          loop_rate_(30.0) 
    {}
    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    LibSerial::SerialPort serial_port_;
    std::string port_name_;
    int baud_rate_;
    int enc_counts_per_revolution_;
    float loop_rate_;

    Wheel left_wheel_;
    Wheel right_wheel_;
    std::chrono::time_point<std::chrono::system_clock> time_;

    rclcpp::Logger logger_;
}; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware

#endif
