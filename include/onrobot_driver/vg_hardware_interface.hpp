#ifndef VG_HW_INTERFACE_HPP
#define VG_HW_INTERFACE_HPP

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "VG.hpp"

namespace onrobot_driver
{

    class VGHardwareInterface : public hardware_interface::ActuatorInterface
    {
    public:
        VGHardwareInterface();
        ~VGHardwareInterface() override;

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
        std::unique_ptr<VG> gripper_;
        std::string prefix_;
        hardware_interface::HardwareInfo info_;

        // Internal joint variables (for vacuum gripper, we simulate a binary state)
        double gripper_state_;   // 0 = released, 1 = gripped
        double gripper_command_; // commanded state

        // Connection parameters
        std::string onrobot_type_;
        std::string connection_type_;
        std::string ip_address_;
        int port_;
        std::string device_;
        int device_address_;
        bool use_fake_hardware_;
        bool auto_detect_;

        // Mutex for thread safety
        std::mutex hw_interface_mutex_;
    };

} // namespace onrobot_driver

#endif // VG_HW_INTERFACE_HPP