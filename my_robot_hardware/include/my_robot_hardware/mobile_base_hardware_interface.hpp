#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include <vector>
#include <string>

namespace mobile_base_hardware {

struct Wheel {
    std::string name = "";
    double position = 0.0;
    double velocity = 0.0;
    double command = 0.0;
};

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
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

    Wheel left_wheel_;
    Wheel right_wheel_;

}; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware

#endif
