#include "onrobot_driver/threefg_hardware_interface.hpp"
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

    // === 1. Validate gripper type ===
    if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing 'onrobot_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    std::string onrobot_type = info.hardware_parameters.at("onrobot_type");
    if (onrobot_type != "3fg15")
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"),
                     "Invalid onrobot_type for 3FG: '%s'. Only 3fg15 supported", onrobot_type.c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Unsupported connection_type for 3FG: %s", connection_type_.c_str());
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

    // === 8. Initialize for 3FG15 ===
    finger_width_state_ = 0.075;  // Half of 150mm diameter
    finger_width_command_ = 0.075;

    RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"),
                "3FG15 Hardware Interface initialized (fake: %s, prefix: '%s')",
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
        if (connection_type_ == "tcp") {
            RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                       "Creating TCP connection to %s:%d", ip_address_.c_str(), port_);
            gripper_ = std::make_unique<ThreeFG>(ip_address_, port_);
        }
        else if (connection_type_ == "serial") {
            RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                       "Creating Serial connection to %s", device_.c_str());
            gripper_ = std::make_unique<ThreeFG>(device_);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                        "Unsupported connection type for 3FG15: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Read initial diameter
        float diameter = gripper_->getWidth();
        if (diameter < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to read initial diameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        finger_width_state_ = diameter;
        finger_width_command_ = diameter;

        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"),
                    "3FG15 configured. Initial diameter: %.3f m", diameter);
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
        finger_width_state_ += (finger_width_command_ - finger_width_state_) * 0.1f;
        return hardware_interface::return_type::OK;
    }

    if (!gripper_) return hardware_interface::return_type::ERROR;

    try
    {
        float diameter = gripper_->getWidth();
        if (diameter >= 0.0f) finger_width_state_ = diameter;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Read error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThreeFGHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);

    if (use_fake_hardware_) return hardware_interface::return_type::OK;
    if (!gripper_) return hardware_interface::return_type::ERROR;

    // Validate range for 3FG15 (0.0 to 0.150 m diameter)
    if (finger_width_command_ < 0.0 || finger_width_command_ > 0.150)
    {
        RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"),
                    "Command %.3f m out of range [0.0, 0.150] for 3FG15",
                    finger_width_command_);
        return hardware_interface::return_type::OK;
    }

    if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
    {
        try
        {
            gripper_->moveGripper(finger_width_command_);
            RCLCPP_DEBUG(rclcpp::get_logger("ThreeFGHardwareInterface"),
                         "Sent command: %.3f m (3FG15 diameter)", finger_width_command_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Write error: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::ThreeFGHardwareInterface, hardware_interface::ActuatorInterface)
