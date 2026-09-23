#include "onrobot_driver/common/OnRobotHardwareInterfaceBase.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

OnRobotHardwareInterfaceBase::OnRobotHardwareInterfaceBase() = default;

OnRobotHardwareInterfaceBase::~OnRobotHardwareInterfaceBase()
{
    stopAsyncWorker();
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_init(const hardware_interface::HardwareInfo &info)
{
    info_ = info;

    // 1. Connection type
    if (info.hardware_parameters.find("connection_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Missing 'connection_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    connection_type_ = info.hardware_parameters.at("connection_type");

    // 2. Connection parameters
    if (connection_type_ == "tcp")
    {
        if (info.hardware_parameters.find("ip_address") == info.hardware_parameters.end() ||
            info.hardware_parameters.find("port") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Missing ip_address or port for TCP");
            return hardware_interface::CallbackReturn::ERROR;
        }
        ip_address_ = info.hardware_parameters.at("ip_address");
        port_ = std::stoi(info.hardware_parameters.at("port"));
    }
    else if (connection_type_ == "serial")
    {
        if (info.hardware_parameters.find("device") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Missing 'device' for serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        device_ = info.hardware_parameters.at("device");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3. Device address
    if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
    {
        device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
    }

    // 4. Fake hardware mode
    if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
    {
        use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
    }

    // 5. Prefix
    if (info.hardware_parameters.find("prefix") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Missing 'prefix' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    prefix_ = info.hardware_parameters.at("prefix");

    // 6. Validate joint
    if (info.joints.size() != 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Expected 1 joint, got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info.joints[0].name != "finger_width")
    {
        RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Expected joint 'finger_width', got '%s'",
                     info.joints[0].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_configure(const rclcpp_lifecycle::State &)
{
    if (use_fake_hardware_)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Using fake hardware for %s", onrobot_type_.c_str());
        finger_width_state_ = max_width_ / 2.0;
        finger_width_command_ = finger_width_state_;
        finger_width_velocity_ = 0.0;
        finger_width_effort_ = 0.0;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    try
    {
        if (!instantiateGripper())
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Failed to instantiate gripper %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        OnRobotGripperBase *gripper = getGripperBase();
        if (!gripper)
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Gripper instance is null");
            return hardware_interface::CallbackReturn::ERROR;
        }

        float initial_width = gripper->getWidth();
        if (initial_width < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Failed to read initial width from %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        finger_width_state_ = initial_width;
        finger_width_command_ = initial_width;
        finger_width_velocity_ = 0.0;
        finger_width_effort_ = 0.0;

        {
            std::lock_guard<std::mutex> lock(async_state_mutex_);
            cached_position_ = initial_width;
            cached_velocity_ = 0.0;
            cached_effort_ = 0.0;
            cached_status_ = gripper->getStatusRaw();
        }

        {
            std::lock_guard<std::mutex> lock(async_cmd_mutex_);
            desired_position_ = initial_width;
            desired_effort_ = max_force_ / 2.0;
            new_cmd_available_ = false;
        }

        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"),
                    "%s configured. Initial width: %.3f m. Limits: [%.3f, %.3f] m, Max force: %.1f N",
                    onrobot_type_.c_str(), initial_width, min_width_, max_width_, max_force_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Failed to configure %s: %s", onrobot_type_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_activate(const rclcpp_lifecycle::State &)
{
    if (!use_fake_hardware_)
    {
        startAsyncWorker();
    }
    RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "%s activated (async worker running)", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_deactivate(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "%s deactivated", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_cleanup(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    destroyGripper();
    RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "%s cleaned up", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_shutdown(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    destroyGripper();
    RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "%s shutdown", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OnRobotHardwareInterfaceBase::on_error(const rclcpp_lifecycle::State &)
{
    stopAsyncWorker();
    RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "%s entered error state", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OnRobotHardwareInterfaceBase::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "velocity", &finger_width_velocity_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "effort", &finger_width_effort_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OnRobotHardwareInterfaceBase::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "position", &finger_width_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "max_effort", &finger_width_effort_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "effort", &finger_width_effort_command_));
    return command_interfaces;
}

hardware_interface::return_type OnRobotHardwareInterfaceBase::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
    double dt = period.seconds();
    if (dt <= 0.0)
    {
        dt = 0.02; // Default 50 Hz fallback
    }

    if (use_fake_hardware_)
    {
        double prev_pos = finger_width_state_;
        float movement = static_cast<float>((finger_width_command_ - finger_width_state_) * 0.1);
        finger_width_state_ += movement;
        finger_width_velocity_ = (finger_width_state_ - prev_pos) / dt;
        finger_width_effort_ = (std::abs(finger_width_command_ - finger_width_state_) < 0.001) ? finger_width_effort_command_ : 0.0;
        return hardware_interface::return_type::OK;
    }

    // Real-Time Decoupled Read: Instantaneous memory read without waiting on network bus
    {
        std::lock_guard<std::mutex> lock(async_state_mutex_);
        finger_width_state_ = cached_position_;
        finger_width_velocity_ = cached_velocity_;
        finger_width_effort_ = cached_effort_;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type OnRobotHardwareInterfaceBase::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (use_fake_hardware_)
    {
        return hardware_interface::return_type::OK;
    }

    // Range clamping
    if (finger_width_command_ < min_width_ || finger_width_command_ > max_width_)
    {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OnRobotHardwareInterface"),
                             clock_, 2000,
                             "Command %.3f m out of range [%.3f, %.3f] for %s",
                             finger_width_command_, min_width_, max_width_, onrobot_type_.c_str());
        return hardware_interface::return_type::OK;
    }

    // Real-Time Decoupled Write: update atomic command and notify background thread without blocking
    {
        std::lock_guard<std::mutex> lock(async_cmd_mutex_);
        desired_position_ = finger_width_command_;
        desired_effort_ = finger_width_effort_command_;
        new_cmd_available_ = true;
    }

    return hardware_interface::return_type::OK;
}

void OnRobotHardwareInterfaceBase::startAsyncWorker()
{
    if (!worker_running_)
    {
        worker_running_ = true;
        async_worker_thread_ = std::thread(&OnRobotHardwareInterfaceBase::asyncWorkerLoop, this);
    }
}

void OnRobotHardwareInterfaceBase::stopAsyncWorker()
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

void OnRobotHardwareInterfaceBase::asyncWorkerLoop()
{
    OnRobotGripperBase *gripper = getGripperBase();
    if (!gripper)
    {
        return;
    }

    double last_position = cached_position_;
    auto last_time = std::chrono::steady_clock::now();
    double last_sent_position = -1.0;
    double last_sent_effort = -1.0;

    // Asynchronous communication loop decoupled from RT controller manager
    while (worker_running_)
    {
        auto loop_start = std::chrono::steady_clock::now();

        try
        {
            // 1. Process pending write commands
            double cmd_pos = 0.0;
            double cmd_eff = 0.0;
            bool has_new_cmd = false;

            {
                std::lock_guard<std::mutex> lock(async_cmd_mutex_);
                if (new_cmd_available_)
                {
                    cmd_pos = desired_position_;
                    cmd_eff = desired_effort_;
                    has_new_cmd = true;
                    new_cmd_available_ = false;
                }
            }

            if (has_new_cmd)
            {
                if (cmd_eff > 0.0 && std::abs(cmd_eff - last_sent_effort) > 0.5)
                {
                    gripper->setTargetForce(static_cast<float>(cmd_eff));
                    last_sent_effort = cmd_eff;
                }

                if (std::abs(cmd_pos - last_sent_position) > 0.0005)
                {
                    gripper->moveGripper(static_cast<float>(cmd_pos));
                    last_sent_position = cmd_pos;
                }
            }

            // 2. Read state from physical hardware
            float curr_width = gripper->getWidth();
            float curr_force = gripper->getForce();
            uint16_t curr_status = gripper->getStatusRaw();

            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            if (dt <= 0.0)
            {
                dt = 0.03;
            }

            double curr_velocity = 0.0;
            if (curr_width >= 0.0f)
            {
                curr_velocity = (static_cast<double>(curr_width) - last_position) / dt;
                last_position = static_cast<double>(curr_width);
            }
            last_time = now;

            // 3. Update state cache
            {
                std::lock_guard<std::mutex> lock(async_state_mutex_);
                if (curr_width >= 0.0f)
                {
                    cached_position_ = static_cast<double>(curr_width);
                    cached_velocity_ = curr_velocity;
                }
                if (curr_force >= 0.0f)
                {
                    cached_effort_ = static_cast<double>(curr_force);
                }
                cached_status_ = curr_status;
                comm_healthy_ = true;
            }
        }
        catch (const std::exception &ex)
        {
            comm_healthy_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            try
            {
                if (gripper->reconnect())
                {
                    comm_healthy_ = true;
                    last_sent_position = -1.0;
                    last_sent_effort = -1.0;
                }
            }
            catch (...)
            {
            }
        }

        // Target async loop rate ~35 Hz (approx 28 ms)
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loop_start);
        if (elapsed < std::chrono::milliseconds(30))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30) - elapsed);
        }
    }
}

} // namespace onrobot_driver
