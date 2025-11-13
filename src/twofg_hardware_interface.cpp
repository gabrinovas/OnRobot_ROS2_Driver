#include "onrobot_driver/twofg_hardware_interface.hpp"
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

    TwoFGHardwareInterface::~TwoFGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        info_ = info; // Store the hardware info
        
        // Retrieve the gripper type from the hardware parameters
        if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end())
        {
            onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Check for valid gripper type for 2FG series
        if (onrobot_type_ != "2fg7" && onrobot_type_ != "2fg14")
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Invalid onrobot_type for 2FG series: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the connection type from the hardware parameters
        if (info.hardware_parameters.find("connection_type") != info.hardware_parameters.end())
        {
            connection_type_ = info.hardware_parameters.at("connection_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing connection_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Check for TCP connection parameters
        if (connection_type_ == "tcp")
        {
            if (info.hardware_parameters.find("ip_address") != info.hardware_parameters.end() &&
                info.hardware_parameters.find("port") != info.hardware_parameters.end())
            {
                ip_address_ = info.hardware_parameters.at("ip_address");
                port_ = std::stoi(info.hardware_parameters.at("port"));
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing ip_address or port for TCP connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // Check for Serial connection parameters
        else if (connection_type_ == "serial")
        {
            if (info.hardware_parameters.find("device") != info.hardware_parameters.end())
            {
                device_ = info.hardware_parameters.at("device");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve device address
        if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
        {
            device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
        }

        // Retrieve use_fake_hardware parameter
        if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
        {
            use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
        }

        // Retrieve the prefix from the hardware parameters
        if (info.hardware_parameters.find("prefix") != info.hardware_parameters.end())
        {
            prefix_ = info.hardware_parameters.at("prefix");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate the hardware info
        if (info.joints.size() != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Expected exactly 1 joint, got %zu", info.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.joints[0].name != "finger_width") {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Expected joint name 'finger_width', got '%s'", info.joints[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize joint variables
        finger_width_state_ = 0.035; // Start at half-open
        finger_width_command_ = 0.035;
        
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "2FG Hardware Interface initialized for %s (fake hardware: %s)", 
                   onrobot_type_.c_str(), use_fake_hardware_ ? "true" : "false");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        // Only create real hardware interface if not using fake hardware
        if (use_fake_hardware_)
        {
            RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "Using fake hardware for 2FG gripper");
            finger_width_state_ = 0.035; // Start at half-open
            finger_width_command_ = 0.035;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // Create the TwoFG instance for real hardware
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
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), 
                            "Unsupported connection type: %s", connection_type_.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Test connection by reading initial width
            float initial_width = gripper_->getWidth();
            if (initial_width < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), 
                            "Failed to read initial gripper width");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            finger_width_state_ = initial_width;
            finger_width_command_ = initial_width;
            
            RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), 
                       "2FG gripper configured. Initial width: %.3f m", finger_width_state_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), 
                        "Failed to create TwoFG instance: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "2FG Hardware Interface cleaned up");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "2FG Hardware Interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "2FG Hardware Interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("TwoFGHardwareInterface"), "2FG Hardware Interface shutdown");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TwoFGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "2FG Hardware Interface error occurred");
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
            // For fake hardware, simulate gripper movement
            float movement = (finger_width_command_ - finger_width_state_) * 0.1f;
            finger_width_state_ += movement;
            return hardware_interface::return_type::OK;
        }
        
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Gripper not initialized");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            float new_width = gripper_->getWidth();
            if (new_width >= 0.0) {
                finger_width_state_ = new_width;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to read gripper width");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to read gripper state: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TwoFGHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        
        if (use_fake_hardware_)
        {
            // For fake hardware, just accept the command
            return hardware_interface::return_type::OK;
        }
        
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Gripper not initialized");
            return hardware_interface::return_type::ERROR;
        }
        
        // Only send command if it has changed significantly
        if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
        {
            try
            {
                gripper_->moveGripper(finger_width_command_);
                RCLCPP_DEBUG(rclcpp::get_logger("TwoFGHardwareInterface"), "Commanded gripper to width: %.3f m", finger_width_command_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace onrobot_driver

// Export the hardware interface as a plugin for ros2_control
PLUGINLIB_EXPORT_CLASS(onrobot_driver::TwoFGHardwareInterface, hardware_interface::ActuatorInterface)
