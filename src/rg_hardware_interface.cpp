#include "onrobot_driver/rg_hardware_interface.hpp"
#include "onrobot_driver/GripperDetection.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rg_hardware_interface
{

    RGHardwareInterface::RGHardwareInterface()
        : finger_width_state_(0.0),
          finger_width_command_(0.0),
          device_address_(65),
          use_fake_hardware_(false),
          auto_detect_(false)  // Initialize auto_detect
    {
    }

    RGHardwareInterface::~RGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        info_ = info; // Store the hardware info
        
        // Retrieve the gripper type from the hardware parameters.
        if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end())
        {
            onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Check for valid gripper type.
        if (onrobot_type_ != "rg2" && onrobot_type_ != "rg6")
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Invalid onrobot_type: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve auto_detect parameter
        if (info.hardware_parameters.find("auto_detect") != info.hardware_parameters.end())
        {
            auto_detect_ = (info.hardware_parameters.at("auto_detect") == "true");
        }

        // Retrieve the connection type from the hardware parameters.
        if (info.hardware_parameters.find("connection_type") != info.hardware_parameters.end())
        {
            connection_type_ = info.hardware_parameters.at("connection_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing connection_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Check for TCP connection parameters.
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
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing ip_address or port for TCP connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // Check for Serial connection parameters.
        else if (connection_type_ == "serial")
        {
            if (info.hardware_parameters.find("device") != info.hardware_parameters.end())
            {
                device_ = info.hardware_parameters.at("device");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
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
        } else {
            // Default device address for RG series is 65
            device_address_ = 65;
        }

        // Retrieve use_fake_hardware parameter
        if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
        {
            use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
        }

        // Retrieve the prefix from the hardware parameters.
        if (info.hardware_parameters.find("prefix") != info.hardware_parameters.end())
        {
            prefix_ = info.hardware_parameters.at("prefix");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialise joint variables
        if (onrobot_type_ == "rg2") {
            finger_width_state_ = 0.055; // Start at half-open for RG2
        } else {
            finger_width_state_ = 0.08; // Start at half-open for RG6
        }
        finger_width_command_ = finger_width_state_;
        
        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                   "RG Hardware Interface initialized for %s (auto_detect: %s, fake hardware: %s)", 
                   onrobot_type_.c_str(),
                   auto_detect_ ? "true" : "false",
                   use_fake_hardware_ ? "true" : "false");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        // Check for fake hardware parameter
        if (use_fake_hardware_)
        {
            RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), "Using fake hardware for RG gripper");
            if (onrobot_type_ == "rg2") {
                finger_width_state_ = 0.055; // Start at half-open for RG2
            } else {
                finger_width_state_ = 0.08; // Start at half-open for RG6
            }
            finger_width_command_ = finger_width_state_;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // Create the RG instance for real hardware
        try
        {
            std::unique_ptr<IModbusConnection> temp_connection;
            
            // Create temporary connection for detection
            if (connection_type_ == "tcp")
            {
                temp_connection = std::make_unique<TCPConnectionWrapper>(ip_address_, port_);
            }
            else if (connection_type_ == "serial")
            {
                temp_connection = std::make_unique<SerialConnectionWrapper>(device_);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), 
                            "Unsupported connection type: %s", connection_type_.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Auto-detect gripper type if enabled
            if (auto_detect_) {
                std::string detected_type = GripperDetection::detectGripperType(temp_connection, device_address_);
                
                if (!detected_type.empty()) {
                    if (detected_type != onrobot_type_) {
                        RCLCPP_WARN(rclcpp::get_logger("RGHardwareInterface"), 
                                   "Auto-detected gripper type: %s (configured: %s)", 
                                   detected_type.c_str(), onrobot_type_.c_str());
                        
                        // Check if detected type is compatible with this hardware interface
                        if (detected_type.find("rg") == std::string::npos) {
                            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), 
                                       "Detected gripper type %s is not an RG series gripper. "
                                       "This hardware interface only supports RG series.",
                                       detected_type.c_str());
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                        
                        onrobot_type_ = detected_type;
                    } else {
                        RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                                   "Auto-detection confirmed configured type: %s", onrobot_type_.c_str());
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("RGHardwareInterface"), 
                               "Auto-detection failed. Using configured type: %s", onrobot_type_.c_str());
                }
            }

            // Now create the actual gripper instance
            if (connection_type_ == "tcp")
            {
                RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                           "Creating TCP connection to %s:%d, device address: %d", 
                           ip_address_.c_str(), port_, device_address_);
                gripper_ = std::make_unique<RG>(onrobot_type_, ip_address_, port_);
            }
            else if (connection_type_ == "serial")
            {
                RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                           "Creating Serial connection to %s, device address: %d", 
                           device_.c_str(), device_address_);
                gripper_ = std::make_unique<RG>(onrobot_type_, device_);
            }

            // Get the starting width of the gripper.
            finger_width_state_ = gripper_->getWidthWithOffset();
            // Set the command to the current state to avoid moving to 0.0 at start.
            finger_width_command_ = finger_width_state_;
            
            RCLCPP_INFO(rclcpp::get_logger("RGHardwareInterface"), 
                       "RG gripper configured. Type: %s, Initial width: %.3f m", 
                       onrobot_type_.c_str(), finger_width_state_);
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
        
        // Check for fake hardware
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
            finger_width_state_ = gripper_->getWidthWithOffset();
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
        
        // Check for fake hardware
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
        try
        {
            gripper_->moveGripper(finger_width_command_);
            RCLCPP_DEBUG(rclcpp::get_logger("RGHardwareInterface"), "Commanded gripper to width: %.3f m", finger_width_command_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

} // namespace rg_hardware_interface

// Export the hardware interface as a plugin for ros2_control.
PLUGINLIB_EXPORT_CLASS(rg_hardware_interface::RGHardwareInterface, hardware_interface::ActuatorInterface)
