#include "onrobot_driver/twofg/twofg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

TwoFGHardwareInterface::TwoFGHardwareInterface()
    : finger_width_state_(0.0),
      finger_width_command_(0.0),
      device_address_(65),
      use_fake_hardware_(false)
{
}

TwoFGHardwareInterface::~TwoFGHardwareInterface() = default;

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    info_ = info;

    // === 1. Validate it's a 2FG gripper ===
    if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing 'onrobot_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    onrobot_type_ = info.hardware_parameters.at("onrobot_type");
    if (onrobot_type_ != "2fg7" && onrobot_type_ != "2fg14")
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), 
                    "Invalid onrobot_type for TwoFG: '%s'. Expected '2fg7' or '2fg14'", 
                    onrobot_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 2. Connection type ===
    if (info.hardware_parameters.find("connection_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing 'connection_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    connection_type_ = info.hardware_parameters.at("connection_type");

    // === 3. Connection parameters ===
    if (connection_type_ == "tcp")
    {
        if (info.hardware_parameters.find("ip_address") == info.hardware_parameters.end() ||
            info.hardware_parameters.find("port") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing ip_address or port for TCP");
            return hardware_interface::CallbackReturn::ERROR;
        }
        ip_address_ = info.hardware_parameters.at("ip_address");
        port_ = std::stoi(info.hardware_parameters.at("port"));
    }
    else if (connection_type_ == "serial")
    {
        if (info.hardware_parameters.find("device") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing 'device' for serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        device_ = info.hardware_parameters.at("device");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 4. Device address ===
    if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
    {
        device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
    }

    // === 5. Fake hardware mode ===
    if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
    {
        use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
    }

    // === 6. Prefix ===
    if (info.hardware_parameters.find("prefix") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing 'prefix' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    prefix_ = info.hardware_parameters.at("prefix");

    // === 7. Validate joint ===
    if (info.joints.size() != 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Expected 1 joint, got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info.joints[0].name != "finger_width")
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Expected joint 'finger_width', got '%s'",
                     info.joints[0].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 8. Initialize joint variables based on 2FG type ===
    if (onrobot_type_ == "2fg14")
    {
        finger_width_state_ = 0.070;  // Half of 140mm for 2FG14
        finger_width_command_ = 0.070;
    }
    else
    {
        finger_width_state_ = 0.035;  // Half of 70mm for 2FG7
        finger_width_command_ = 0.035;
    }

    RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"),
                "OnRobot %s Hardware Interface initialized (fake: %s, prefix: '%s')",
                onrobot_type_.c_str(), use_fake_hardware_ ? "true" : "false", prefix_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    if (use_fake_hardware_)
    {
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "Using fake hardware for %s", onrobot_type_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    try
    {
        if (connection_type_ == "tcp")
        {
            RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), 
                       "Creating TCP connection to %s:%d, device address: %d", 
                       ip_address_.c_str(), port_, device_address_);
            gripper_ = std::make_unique<TwoFG>(onrobot_type_, ip_address_, port_, device_address_);
        }
        else if (connection_type_ == "serial")
        {
            RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), 
                       "Creating Serial connection to %s, device address: %d", 
                       device_.c_str(), device_address_);
            gripper_ = std::make_unique<TwoFG>(onrobot_type_, device_, device_address_);
        }

        // Test connection by reading initial width
        float initial_width = gripper_->getWidth();
        if (initial_width < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to read initial width");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        finger_width_state_ = initial_width;
        finger_width_command_ = initial_width;

        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"),
                   "%s configured. Initial width: %.3f m", onrobot_type_.c_str(), initial_width);

        // Log 2FG specific capabilities
        float min_width = gripper_->getMinWidth();
        float max_width = gripper_->getMaxWidth();
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"),
                   "%s range: %.3f m to %.3f m", onrobot_type_.c_str(), min_width, max_width);

    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to configure %s gripper: %s", 
                    onrobot_type_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "%s Hardware Interface cleaned up", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "%s activated", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "%s deactivated", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "%s shutdown", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "%s error state", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TwoFGHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "velocity", &finger_width_state_)); // Dummy velocity
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TwoFGHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "position", &finger_width_command_));
    return command_interfaces;
}

hardware_interface::return_type TwoFGHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);

    if (use_fake_hardware_)
    {
        // Simulate gripper movement for fake hardware
        float movement = (finger_width_command_ - finger_width_state_) * 0.1f;
        finger_width_state_ += movement;
        return hardware_interface::return_type::OK;
    }

    if (!gripper_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "%s gripper not initialized", onrobot_type_.c_str());
        return hardware_interface::return_type::ERROR;
    }

    try
    {
        float current_width = gripper_->getWidth();
        if (current_width >= 0.0f)
        {
            finger_width_state_ = current_width;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to read %s width", onrobot_type_.c_str());
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to read %s state: %s", onrobot_type_.c_str(), e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type TwoFGHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);

    if (use_fake_hardware_)
    {
        return hardware_interface::return_type::OK;
    }

    if (!gripper_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "%s gripper not initialized", onrobot_type_.c_str());
        return hardware_interface::return_type::ERROR;
    }

    // 2FG series range validation
    double max_width = (onrobot_type_ == "2fg14") ? 0.140 : 0.070;
    if (finger_width_command_ < 0.0 || finger_width_command_ > max_width)
    {
        RCLCPP_WARN(rclcpp::get_logger("TwoFGHardwareInterface"),
                   "Command %.3f m out of range [0.0, %.3f] for %s",
                   finger_width_command_, max_width, onrobot_type_.c_str());
        return hardware_interface::return_type::OK;
    }

    // Only send command if it has changed significantly
    if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
    {
        try
        {
            // 2FG series uses moveGripper with external grip by default
            gripper_->moveGripper(finger_width_command_, true); // true = external grip
            RCLCPP_DEBUG(rclcpp::get_logger("TwoFGHardwareInterface"),
                        "Sent %s command: %.3f m width", onrobot_type_.c_str(), finger_width_command_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to write command to %s: %s", 
                        onrobot_type_.c_str(), e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::TwoFGHardwareInterface, hardware_interface::ActuatorInterface)