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
#include "realtime_tools/realtime_buffer.hpp"

#include "onrobot_driver/vgc10/VGC10.hpp"

namespace onrobot_driver
{

struct VGC10StateData
{
    double vacuum_a{0.0};
    double effort_a{0.0};
    double vacuum_b{0.0};
    double effort_b{0.0};
    bool healthy{true};
};

class VGC10HardwareInterface : public hardware_interface::ActuatorInterface
{
public:
    VGC10HardwareInterface();
    ~VGC10HardwareInterface() override;

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

    // Real-time deterministic read and write
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::unique_ptr<VGC10> gripper_;

    // Parameters
    std::string onrobot_type_{"vgc10"};
    std::string prefix_{""};
    std::string connection_type_{"tcp"};
    std::string ip_address_{"192.168.1.1"};
    int port_{502};
    std::string device_{"/tmp/ttyUR"};
    int device_address_{65};
    bool use_fake_hardware_{false};

    // Joint names
    std::string joint_name_a_;
    std::string joint_name_b_;

    // RT joint variables (Channel A)
    double vacuum_a_state_{0.0};
    double vacuum_a_velocity_{0.0};
    double vacuum_a_effort_{0.0};
    double vacuum_a_level_state_{0.0};
    double pressure_a_kpa_state_{0.0};
    double vacuum_a_command_{0.0};
    double vacuum_a_effort_command_{60.0};

    // RT joint variables (Channel B)
    double vacuum_b_state_{0.0};
    double vacuum_b_velocity_{0.0};
    double vacuum_b_effort_{0.0};
    double vacuum_b_level_state_{0.0};
    double pressure_b_kpa_state_{0.0};
    double vacuum_b_command_{0.0};
    double vacuum_b_effort_command_{60.0};

    // Asynchronous communication worker thread members
    void startAsyncWorker();
    void stopAsyncWorker();
    void asyncWorkerLoop();

    std::thread async_worker_thread_;
    std::atomic<bool> worker_running_{false};
    std::atomic<bool> comm_healthy_{true};

    // Lock-free real-time state buffer: worker writes, RT loop reads (O(1) wait-free / lock-free)
    realtime_tools::RealtimeBuffer<VGC10StateData> state_buffer_;

    // Wait-free atomic command buffer from RT loop to worker thread
    std::atomic<double> desired_vacuum_a_{0.0};
    std::atomic<double> desired_vacuum_b_{0.0};
    std::atomic<double> desired_effort_a_{60.0};
    std::atomic<double> desired_effort_b_{60.0};
    std::atomic<bool> new_cmd_available_{false};
};

} // namespace onrobot_driver
