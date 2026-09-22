#pragma once

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "OnRobotGripperBase.hpp"

namespace onrobot_driver
{

class OnRobotHardwareInterfaceBase : public hardware_interface::ActuatorInterface
{
public:
    OnRobotHardwareInterfaceBase();
    ~OnRobotHardwareInterfaceBase() override;

    // Common lifecycle methods
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

    // Deterministic, non-blocking real-time read and write methods
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    // Factory method to be implemented by derived classes
    virtual bool instantiateGripper() = 0;
    virtual void destroyGripper() = 0;
    virtual OnRobotGripperBase *getGripperBase() = 0;

    // Model limits configured by derived classes
    double min_width_{0.0};
    double max_width_{0.070};
    double max_force_{70.0};
    std::string onrobot_type_{"unknown"};

    // Hardware parameters
    std::string connection_type_;
    std::string ip_address_;
    int port_{502};
    std::string device_;
    int device_address_{65};
    bool use_fake_hardware_{false};
    std::string prefix_;
    hardware_interface::HardwareInfo info_;

    // RT joint variables (accessed exclusively on RT thread)
    double finger_width_state_{0.035};
    double finger_width_velocity_{0.0};
    double finger_width_effort_{0.0};
    double finger_width_command_{0.035};
    double finger_width_effort_command_{35.0};

    // Asynchronous communication worker thread members
    void startAsyncWorker();
    void stopAsyncWorker();
    void asyncWorkerLoop();

    std::thread async_worker_thread_;
    std::atomic<bool> worker_running_{false};
    std::atomic<bool> comm_healthy_{true};

    // Shared state cache between async worker and RT loop
    std::mutex async_state_mutex_;
    double cached_position_{0.035};
    double cached_velocity_{0.0};
    double cached_effort_{0.0};
    uint16_t cached_status_{0};

    // Shared command cache from RT loop to async worker
    std::mutex async_cmd_mutex_;
    double desired_position_{0.035};
    double desired_effort_{35.0};
    std::atomic<bool> new_cmd_available_{false};

    rclcpp::Clock clock_{RCL_STEADY_TIME};
};

} // namespace onrobot_driver
