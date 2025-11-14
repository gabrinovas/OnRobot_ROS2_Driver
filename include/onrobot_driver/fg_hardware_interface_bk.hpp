#ifndef FG_HW_INTERFACE_HPP
#define FG_HW_INTERFACE_HPP

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "FG.hpp"

namespace onrobot_driver
{

    class FGHardwareInterface : public hardware_interface::ActuatorInterface
    {
    public:
        FGHardwareInterface();
        ~FGHardwareInterface() override;

        // Lifecycle methods
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        // Export hardware interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Read and write methods
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // The gripper instance
        std::unique_ptr<FG> gripper_;
        std::string prefix_;
        hardware_interface::HardwareInfo info_;  // Store hardware info for parameter access

        // Internal joint variables
        double finger_width_state_;   // measured state (m)
        double finger_width_command_; // commanded position (m)

        // Connection parameters
        std::string onrobot_type_;
        std::string connection_type_;
        std::string ip_address_;
        int port_;
        std::string device_;
        int device_address_;
        bool use_fake_hardware_;

        // Mutex for thread safety
        std::mutex hw_interface_mutex_;
    };

} // namespace fg_hardware_interface

#endif // FG_HW_INTERFACE_HPP