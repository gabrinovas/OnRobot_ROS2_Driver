#include "onrobot_driver/vgc10/vgc10_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <algorithm>

namespace onrobot_driver
{

VGC10HardwareInterface::VGC10HardwareInterface()
    : onrobot_type_("vgc10"),
      connection_type_("tcp"),
      ip_address_("192.168.1.1"),
      port_(502),
      device_("/tmp/ttyUR"),
      device_address_(65),
      use_fake_hardware_(false)
{
}

VGC10HardwareInterface::~VGC10HardwareInterface()
{
    stopAsyncWorker();
    gripper_.reset();
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    // 1. Gripper type validation
    if (info_.hardware_parameters.find("onrobot_type") != info_.hardware_parameters.end())
    {
        onrobot_type_ = info_.hardware_parameters.at("onrobot_type");
        if (onrobot_type_ != "vgc10")
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGC10HardwareInterface"),
                         "Expected 'onrobot_type' parameter to be 'vgc10', got '%s'", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // 2. Connection parameters
    if (info_.hardware_parameters.find("connection_type") != info_.hardware_parameters.end())
    {
        connection_type_ = info_.hardware_parameters.at("connection_type");
    }
    if (info_.hardware_parameters.find("ip_address") != info_.hardware_parameters.end())
    {
        ip_address_ = info_.hardware_parameters.at("ip_address");
    }
    if (info_.hardware_parameters.find("port") != info_.hardware_parameters.end())
    {
        port_ = std::stoi(info_.hardware_parameters.at("port"));
    }
    if (info_.hardware_parameters.find("device") != info_.hardware_parameters.end())
    {
        device_ = info_.hardware_parameters.at("device");
    }
    if (info_.hardware_parameters.find("device_address") != info_.hardware_parameters.end())
    {
        device_address_ = std::stoi(info_.hardware_parameters.at("device_address"));
    }
    if (info_.hardware_parameters.find("use_fake_hardware") != info_.hardware_parameters.end())
    {
        use_fake_hardware_ = (info_.hardware_parameters.at("use_fake_hardware") == "true");
    }
    if (info_.hardware_parameters.find("prefix") != info_.hardware_parameters.end())
    {
        prefix_ = info_.hardware_parameters.at("prefix");
    }

    joint_name_a_ = prefix_ + "vacuum_channel_a";
    joint_name_b_ = prefix_ + "vacuum_channel_b";

    // 3. Validate joint definitions
    bool found_a = false;
    bool found_b = false;
    for (const auto &joint : info_.joints)
    {
        if (joint.name == joint_name_a_) found_a = true;
        if (joint.name == joint_name_b_) found_b = true;
    }

    if (!found_a || !found_b)
    {
        RCLCPP_WARN(rclcpp::get_logger("VGC10HardwareInterface"),
                    "Expected joints '%s' and '%s' in hardware info",
                    joint_name_a_.c_str(), joint_name_b_.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"),
                "VGC10 hardware interface initialized (type=%s, conn=%s, fake=%s)",
                onrobot_type_.c_str(), connection_type_.c_str(), use_fake_hardware_ ? "true" : "false");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    if (use_fake_hardware_)
    {
        RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"), "Configuring fake hardware for VGC10");
        vacuum_a_state_ = 0.0;
        vacuum_a_command_ = 0.0;
        vacuum_b_state_ = 0.0;
        vacuum_b_command_ = 0.0;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"), "Connecting to physical VGC10 via %s...", connection_type_.c_str());
    try
    {
        if (connection_type_ == "tcp")
        {
            gripper_ = std::make_unique<VGC10>(onrobot_type_, ip_address_, port_, device_address_, []() { return rclcpp::ok(); });
        }
        else if (connection_type_ == "serial")
        {
            gripper_ = std::make_unique<VGC10>(onrobot_type_, device_, device_address_, []() { return rclcpp::ok(); });
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGC10HardwareInterface"), "Unsupported connection type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("VGC10HardwareInterface"), "Failed to instantiate VGC10 driver: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    if (!use_fake_hardware_)
    {
        startAsyncWorker();
    }
    RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"), "VGC10 activated (dual vacuum channels ready)");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    if (gripper_)
    {
        try { gripper_->releaseAll(); } catch (...) {}
    }
    RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"), "VGC10 deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"), "VGC10 cleaned up");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("VGC10HardwareInterface"), "VGC10 shutdown");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VGC10HardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    RCLCPP_ERROR(rclcpp::get_logger("VGC10HardwareInterface"), "VGC10 entered error state");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VGC10HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Channel A
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_a_, "position", &vacuum_a_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_a_, "velocity", &vacuum_a_velocity_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_a_, "effort", &vacuum_a_effort_));

    // Channel B
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_b_, "position", &vacuum_b_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_b_, "velocity", &vacuum_b_velocity_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_b_, "effort", &vacuum_b_effort_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VGC10HardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Channel A
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name_a_, "position", &vacuum_a_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name_a_, "max_effort", &vacuum_a_effort_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name_a_, "effort", &vacuum_a_effort_command_));

    // Channel B
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name_b_, "position", &vacuum_b_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name_b_, "max_effort", &vacuum_b_effort_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name_b_, "effort", &vacuum_b_effort_command_));

    return command_interfaces;
}

hardware_interface::return_type VGC10HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
    double dt = period.seconds();
    if (dt <= 0.0) dt = 0.02;

    if (use_fake_hardware_)
    {
        // Smooth simulated vacuum ramp up/down
        double prev_a = vacuum_a_state_;
        vacuum_a_state_ += (vacuum_a_command_ - vacuum_a_state_) * 0.15;
        vacuum_a_velocity_ = (vacuum_a_state_ - prev_a) / dt;
        vacuum_a_effort_ = vacuum_a_state_ * 80.0; // 0 to 80%

        double prev_b = vacuum_b_state_;
        vacuum_b_state_ += (vacuum_b_command_ - vacuum_b_state_) * 0.15;
        vacuum_b_velocity_ = (vacuum_b_state_ - prev_b) / dt;
        vacuum_b_effort_ = vacuum_b_state_ * 80.0;

        return hardware_interface::return_type::OK;
    }

    // Real-Time decoupled read from cached state
    {
        std::lock_guard<std::mutex> lock(async_state_mutex_);
        vacuum_a_state_ = cached_vacuum_a_;
        vacuum_a_effort_ = cached_effort_a_;
        vacuum_b_state_ = cached_vacuum_b_;
        vacuum_b_effort_ = cached_effort_b_;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type VGC10HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (use_fake_hardware_)
    {
        return hardware_interface::return_type::OK;
    }

    // Clamp command limits [0.0, 1.0]
    double clamped_a = std::max(0.0, std::min(vacuum_a_command_, 1.0));
    double clamped_b = std::max(0.0, std::min(vacuum_b_command_, 1.0));

    {
        std::lock_guard<std::mutex> lock(async_cmd_mutex_);
        desired_vacuum_a_ = clamped_a;
        desired_vacuum_b_ = clamped_b;
        desired_effort_a_ = vacuum_a_effort_command_;
        desired_effort_b_ = vacuum_b_effort_command_;
        new_cmd_available_ = true;
    }

    return hardware_interface::return_type::OK;
}

void VGC10HardwareInterface::startAsyncWorker()
{
    if (!worker_running_)
    {
        worker_running_ = true;
        async_worker_thread_ = std::thread(&VGC10HardwareInterface::asyncWorkerLoop, this);
    }
}

void VGC10HardwareInterface::stopAsyncWorker()
{
    if (worker_running_)
    {
        worker_running_ = false;
        if (async_worker_thread_.joinable())
        {
            async_worker_thread_.join();
        }
    }
}

void VGC10HardwareInterface::asyncWorkerLoop()
{
    if (!gripper_) return;

    double last_sent_a = -1.0;
    double last_sent_b = -1.0;

    while (worker_running_)
    {
        auto loop_start = std::chrono::steady_clock::now();

        try
        {
            // 1. Process pending write commands
            double cmd_a = 0.0;
            double cmd_b = 0.0;
            double eff_a = 60.0;
            double eff_b = 60.0;
            bool has_new_cmd = false;

            {
                std::lock_guard<std::mutex> lock(async_cmd_mutex_);
                if (new_cmd_available_)
                {
                    cmd_a = desired_vacuum_a_;
                    cmd_b = desired_vacuum_b_;
                    eff_a = desired_effort_a_;
                    eff_b = desired_effort_b_;
                    has_new_cmd = true;
                    new_cmd_available_ = false;
                }
            }

            if (has_new_cmd)
            {
                // Channel A control
                if (std::abs(cmd_a - last_sent_a) > 0.02)
                {
                    if (cmd_a > 0.05)
                    {
                        uint8_t target_pct = static_cast<uint8_t>(std::min(eff_a > 0.0 ? eff_a : (cmd_a * 80.0), 80.0));
                        gripper_->gripChannelA(target_pct);
                    }
                    else
                    {
                        gripper_->releaseChannelA();
                    }
                    last_sent_a = cmd_a;
                }

                // Channel B control
                if (std::abs(cmd_b - last_sent_b) > 0.02)
                {
                    if (cmd_b > 0.05)
                    {
                        uint8_t target_pct = static_cast<uint8_t>(std::min(eff_b > 0.0 ? eff_b : (cmd_b * 80.0), 80.0));
                        gripper_->gripChannelB(target_pct);
                    }
                    else
                    {
                        gripper_->releaseChannelB();
                    }
                    last_sent_b = cmd_b;
                }
            }

            // 2. Read telemetry from physical VGC10
            float vac_a = gripper_->getVacuumChannelA();
            float vac_b = gripper_->getVacuumChannelB();

            {
                std::lock_guard<std::mutex> lock(async_state_mutex_);
                cached_vacuum_a_ = vac_a;
                cached_effort_a_ = vac_a * 100.0; // in %
                cached_vacuum_b_ = vac_b;
                cached_effort_b_ = vac_b * 100.0;
            }

            comm_healthy_ = true;
        }
        catch (const std::exception &e)
        {
            comm_healthy_ = false;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("VGC10HardwareInterface"),
                                 *rclcpp::get_current_node()->get_clock(), 2000,
                                 "Modbus communication error with VGC10: %s", e.what());
        }

        // Maintain ~50 Hz update rate for async loop
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loop_start);
        auto sleep_dur = std::chrono::milliseconds(20) - elapsed;
        if (sleep_dur > std::chrono::milliseconds(0))
        {
            std::this_thread::sleep_for(sleep_dur);
        }
    }
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::VGC10HardwareInterface, hardware_interface::ActuatorInterface)
