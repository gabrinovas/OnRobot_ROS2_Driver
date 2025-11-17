#include "onrobot_driver/rg/rg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

    RGHardwareInterface::RGHardwareInterface()
        : finger_width_state_(0.0),
          finger_width_command_(0.0),
          device_address_(65),
          use_fake_hardware_(false)
    {
    }

    RGHardwareInterface::~RGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        info_ = info;
        
        // Retrieve the gripper type from the hardware parameters.
        if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        
        // Check for valid gripper type.
        if (onrobot_type_ != "rg2" && onrobot_type_ != "rg6")
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Invalid onrobot_type: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the connection type from the hardware parameters.
        if (info.hardware_parameters.find("connection_type") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing connection_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        connection_type_ = info.hardware_parameters.at("connection_type");
        
        // Check for TCP connection parameters.
        if (connection_type_ == "tcp")
        {
            if (info.hardware_parameters.find("ip_address") == info.hardware_parameters.end() ||
                info.hardware_parameters.find("port") == info.hardware_parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing ip_address or port for TCP connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
            ip_address_ = info.hardware_parameters.at("ip_address");
            port_ = std::stoi(info.hardware_parameters.at("port"));
        }
        // Check for Serial connection parameters.
        else if (connection_type_ == "serial")
        {
            if (info.hardware_parameters.find("device") == info.hardware_parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
            device_ = info.hardware_parameters.at("device");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve device address
        if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
        {
            device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
        }

        // Retrieve fake hardware parameter
        if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
        {
            use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
        }

        // Retrieve the prefix from the hardware parameters.
        if (info.hardware_parameters.find("prefix") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        prefix_ = info.hardware_parameters.at("prefix");

        // Validate joint
        if (info.joints.size() != 1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Expected 1 joint, got %zu", info.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (info.joints[0].name != "finger_width")
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Expected joint 'finger_width', got '%s'",
                         info.joints[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialise joint variables based on RG type
        if (onrobot_type_ == "rg2") {
            finger_width_state_ = 0.055; // Start at half-open for RG2
        } else {
            finger_width_state_ = 0.08; // Start at half-open for RG6
        }
        finger_width_command_ = finger_width_state_;
        
        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                   "RG Hardware Interface initialized for %s (fake: %s, prefix: '%s')", 
                   onrobot_type_.c_str(), use_fake_hardware_ ? "true" : "false", prefix_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        if (use_fake_hardware_)
        {
            RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), "Using fake hardware for RG gripper");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // Create the RG instance for real hardware
        try
        {
            if (connection_type_ == "tcp")
            {
                RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                           "Creating TCP connection to %s:%d, device address: %d", 
                           ip_address_.c_str(), port_, device_address_);
                // Use explicit construction to avoid ambiguity
                gripper_ = std::unique_ptr<RG>(new RG(onrobot_type_, ip_address_, port_, device_address_));
            }
            else if (connection_type_ == "serial")
            {
                RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                           "Creating Serial connection to %s, device address: %d", 
                           device_.c_str(), device_address_);
                // Use explicit construction to avoid ambiguity
                gripper_ = std::unique_ptr<RG>(new RG(onrobot_type_, device_, device_address_));
            }

            // Get the starting width of the gripper.
            float initial_width = gripper_->getWidthWithOffset();
            if (initial_width < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to read initial width");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            finger_width_state_ = initial_width;
            finger_width_command_ = finger_width_state_;
            
            RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                       "RG %s configured. Initial width: %.3f m", onrobot_type_.c_str(), finger_width_state_);

            // Log RG specific capabilities
            float min_width = gripper_->getMinWidth();
            float max_width = gripper_->getMaxWidth();
            RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"),
                       "RG %s range: %.3f m to %.3f m", onrobot_type_.c_str(), min_width, max_width);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to create RG instance: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), "RG Hardware Interface cleaned up");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), "RG Hardware Interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), "RG Hardware Interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), "RG Hardware Interface shutdown");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "RG Hardware Interface error occurred");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RGHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "velocity", &finger_width_state_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RGHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "position", &finger_width_command_));
        return command_interfaces;
    }

    hardware_interface::return_type RGHardwareInterface::read(const rclcpp::Time &,
                                                              const rclcpp::Duration &)
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
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Gripper not initialised");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            float current_width = gripper_->getWidthWithOffset();
            if (current_width >= 0.0f)
            {
                finger_width_state_ = current_width;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("RGHardwareInterface"), "Failed to read RG width");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to read gripper state: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RGHardwareInterface::write(const rclcpp::Time &,
                                                               const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        
        if (use_fake_hardware_)
        {
            // For fake hardware, just accept the command
            return hardware_interface::return_type::OK;
        }
        
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Gripper not initialised");
            return hardware_interface::return_type::ERROR;
        }

        // RG series range validation
        double max_width = (onrobot_type_ == "rg6") ? 0.160 : 0.110;
        if (finger_width_command_ < 0.0 || finger_width_command_ > max_width)
        {
            RCLCPP_WARN(rclcpp::get_logger("RGHardwareInterface"),
                       "Command %.3f m out of range [0.0, %.3f] for %s",
                       finger_width_command_, max_width, onrobot_type_.c_str());
            return hardware_interface::return_type::OK;
        }

        // Only send command if it has changed significantly
        if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
        {
            try
            {
                gripper_->moveGripper(finger_width_command_);
                RCLCPP_DEBUG(rclcpp::get_logger("RGHardwareInterface"), 
                            "Sent %s command: %.3f m width", onrobot_type_.c_str(), finger_width_command_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace onrobot_driver

// Export the hardware interface as a plugin for ros2_control.
PLUGINLIB_EXPORT_CLASS(onrobot_driver::RGHardwareInterface, hardware_interface::ActuatorInterface)