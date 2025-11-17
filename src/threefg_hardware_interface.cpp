#include "onrobot_driver/threefg/threefg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

ThreeFGHardwareInterface::ThreeFGHardwareInterface()
    : finger_width_state_(0.0),
      finger_width_command_(0.0),
      device_address_(65),
      use_fake_hardware_(false)
{
}

ThreeFGHardwareInterface::~ThreeFGHardwareInterface() = default;

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    info_ = info;

    // === 1. Validate it's a 3FG15 gripper ===
    if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing 'onrobot_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    std::string type = info.hardware_parameters.at("onrobot_type");
    if (type != "3fg15")
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                    "Invalid onrobot_type for ThreeFG: '%s'. Expected '3fg15'", 
                    type.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 2. Connection type ===
    if (info.hardware_parameters.find("connection_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing 'connection_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    connection_type_ = info.hardware_parameters.at("connection_type");

    // === 3. Connection parameters ===
    if (connection_type_ == "tcp")
    {
        if (info.hardware_parameters.find("ip_address") == info.hardware_parameters.end() ||
            info.hardware_parameters.find("port") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing ip_address or port for TCP");
            return hardware_interface::CallbackReturn::ERROR;
        }
        ip_address_ = info.hardware_parameters.at("ip_address");
        port_ = std::stoi(info.hardware_parameters.at("port"));
    }
    else if (connection_type_ == "serial")
    {
        if (info.hardware_parameters.find("device") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing 'device' for serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        device_ = info.hardware_parameters.at("device");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing 'prefix' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    prefix_ = info.hardware_parameters.at("prefix");

    // === 7. Validate joint ===
    if (info.joints.size() != 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Expected 1 joint, got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info.joints[0].name != "finger_width")
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Expected joint 'finger_width', got '%s'",
                     info.joints[0].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 8. Initialize joint variables for 3FG15 ===
    finger_width_state_ = 0.075;  // Start at half of max diameter (150mm)
    finger_width_command_ = 0.075;

    RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"),
                "OnRobot 3FG15 Hardware Interface initialized (fake: %s, prefix: '%s')",
                use_fake_hardware_ ? "true" : "false", prefix_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    if (use_fake_hardware_)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "Using fake hardware for 3FG15");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    try
    {
        if (connection_type_ == "tcp")
        {
            RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                       "Creating TCP connection to %s:%d, device address: %d", 
                       ip_address_.c_str(), port_, device_address_);
            gripper_ = std::unique_ptr<ThreeFG>(new ThreeFG(ip_address_, port_, device_address_));
        }
        else if (connection_type_ == "serial")
        {
            RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                       "Creating Serial connection to %s, device address: %d", 
                       device_.c_str(), device_address_);
            gripper_ = std::unique_ptr<ThreeFG>(new ThreeFG(device_, device_address_));
        }

        // Test connection by reading initial diameter
        float initial_diameter = gripper_->getWidth();
        if (initial_diameter < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to read initial diameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        finger_width_state_ = initial_diameter;
        finger_width_command_ = initial_diameter;

        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"),
                   "3FG15 configured. Initial diameter: %.3f m", initial_diameter);

        // Log 3FG15 specific capabilities
        float min_diam = gripper_->getMinDiameter();
        float max_diam = gripper_->getMaxDiameter();
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"),
                   "3FG15 range: %.3f m to %.3f m", min_diam, max_diam);

    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to configure 3FG15 gripper: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 Hardware Interface cleaned up");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 shutdown");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 error state");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThreeFGHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "velocity", &finger_width_state_)); // Dummy velocity
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThreeFGHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "position", &finger_width_command_));
    return command_interfaces;
}

hardware_interface::return_type ThreeFGHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
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
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 gripper not initialized");
        return hardware_interface::return_type::ERROR;
    }

    try
    {
        float current_diameter = gripper_->getWidth();
        if (current_diameter >= 0.0f)
        {
            finger_width_state_ = current_diameter;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to read 3FG15 diameter");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to read 3FG15 state: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThreeFGHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);

    if (use_fake_hardware_)
    {
        return hardware_interface::return_type::OK;
    }

    if (!gripper_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG15 gripper not initialized");
        return hardware_interface::return_type::ERROR;
    }

    // 3FG15 specific range validation (diameter: 0.0 - 0.150 m)
    const double MAX_DIAMETER = 0.150;
    if (finger_width_command_ < 0.0 || finger_width_command_ > MAX_DIAMETER)
    {
        RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"),
                   "Command %.3f m out of range [0.0, %.3f] for 3FG15",
                   finger_width_command_, MAX_DIAMETER);
        return hardware_interface::return_type::OK;
    }

    // Only send command if it has changed significantly
    if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
    {
        try
        {
            // 3FG15 uses moveGripper with diameter
            gripper_->moveGripper(finger_width_command_);
            RCLCPP_DEBUG(rclcpp::get_logger("ThreeFGHardwareInterface"),
                        "Sent 3FG15 command: %.3f m diameter", finger_width_command_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to write command to 3FG15: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::ThreeFGHardwareInterface, hardware_interface::ActuatorInterface)