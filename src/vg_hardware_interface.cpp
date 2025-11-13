#include "onrobot_driver/vg_hardware_interface.hpp"
#include "onrobot_driver/GripperDetection.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

    VGHardwareInterface::VGHardwareInterface()
        : gripper_state_(0.0),
          gripper_command_(0.0),
          device_address_(65),
          use_fake_hardware_(false),
          auto_detect_(false)
    {
    }

    VGHardwareInterface::~VGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        info_ = info;
        
        if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end())
        {
            onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (onrobot_type_ != "vg10" && onrobot_type_ != "vgc10")
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Invalid onrobot_type for VG series: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve connection parameters
        if (info.hardware_parameters.find("connection_type") != info.hardware_parameters.end())
        {
            connection_type_ = info.hardware_parameters.at("connection_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing connection_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
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
                RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing ip_address or port for TCP connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else if (connection_type_ == "serial")
        {
            if (info.hardware_parameters.find("device") != info.hardware_parameters.end())
            {
                device_ = info.hardware_parameters.at("device");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
        {
            device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
        } else {
            // Default device address for VG series is 65
            device_address_ = 65;
        }

        if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
        {
            use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
        }

        // Retrieve auto_detect parameter
        if (info.hardware_parameters.find("auto_detect") != info.hardware_parameters.end())
        {
            auto_detect_ = (info.hardware_parameters.at("auto_detect") == "true");
        }

        if (info.hardware_parameters.find("prefix") != info.hardware_parameters.end())
        {
            prefix_ = info.hardware_parameters.at("prefix");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate the hardware info - VG uses a different joint name
        if (info.joints.size() != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Expected exactly 1 joint, got %zu", info.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.joints[0].name != "gripper") {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Expected joint name 'gripper', got '%s'", info.joints[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize joint variables - 0.0 = released, 1.0 = gripped
        gripper_state_ = 0.0;
        gripper_command_ = 0.0;
        
        RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), 
                   "VG Hardware Interface initialized for %s (auto_detect: %s, fake hardware: %s)", 
                   onrobot_type_.c_str(), 
                   auto_detect_ ? "true" : "false",
                   use_fake_hardware_ ? "true" : "false");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        if (use_fake_hardware_)
        {
            RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), "Using fake hardware for VG gripper");
            gripper_state_ = 0.0;
            gripper_command_ = 0.0;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // Create the VG instance for real hardware
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
                RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), 
                            "Unsupported connection type: %s", connection_type_.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Auto-detect gripper type if enabled
            if (auto_detect_) {
                std::string detected_type = GripperDetection::detectGripperType(temp_connection, device_address_);
                
                if (!detected_type.empty()) {
                    if (detected_type != onrobot_type_) {
                        RCLCPP_WARN(rclcpp::get_logger("VGHardwareInterface"), 
                                   "Auto-detected gripper type: %s (configured: %s)", 
                                   detected_type.c_str(), onrobot_type_.c_str());
                        
                        // Check if detected type is compatible with this hardware interface
                        if (detected_type.find("vg") == std::string::npos) {
                            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), 
                                       "Detected gripper type %s is not a VG series gripper. "
                                       "This hardware interface only supports VG series.",
                                       detected_type.c_str());
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                        
                        onrobot_type_ = detected_type;
                    } else {
                        RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), 
                                   "Auto-detection confirmed configured type: %s", onrobot_type_.c_str());
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("VGHardwareInterface"), 
                               "Auto-detection failed. Using configured type: %s", onrobot_type_.c_str());
                }
            }

            // Now create the actual gripper instance
            if (connection_type_ == "tcp")
            {
                RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), 
                           "Creating TCP connection to %s:%d, device address: %d", 
                           ip_address_.c_str(), port_, device_address_);
                gripper_ = std::make_unique<VG>(onrobot_type_, ip_address_, port_, device_address_);
            }
            else if (connection_type_ == "serial")
            {
                RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), 
                           "Creating Serial connection to %s, device address: %d", 
                           device_.c_str(), device_address_);
                gripper_ = std::make_unique<VG>(onrobot_type_, device_, device_address_);
            }

            // Set default current limit
            gripper_->setCurrentLimit(500); // 500mA default
            
            // Start in released state
            gripper_->release();
            
            RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), 
                       "VG gripper configured. Type: %s", onrobot_type_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), 
                        "Failed to create VG instance: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            // Release before cleanup
            try {
                gripper_->release();
            } catch (...) {
                // Ignore errors during cleanup
            }
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), "VG Hardware Interface cleaned up");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), "VG Hardware Interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Release when deactivating for safety
        if (gripper_ && !use_fake_hardware_)
        {
            try {
                gripper_->release();
                gripper_state_ = 0.0;
            } catch (...) {
                // Ignore errors during deactivation
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), "VG Hardware Interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            // Release before shutdown for safety
            try {
                gripper_->release();
            } catch (...) {
                // Ignore errors during shutdown
            }
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("VGHardwareInterface"), "VG Hardware Interface shutdown");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "VG Hardware Interface error occurred");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> VGHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "gripper", "position", &gripper_state_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> VGHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "gripper", "position", &gripper_command_));
        return command_interfaces;
    }

    hardware_interface::return_type VGHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        
        if (use_fake_hardware_)
        {
            // For fake hardware, just update state to match command
            // Simulate some delay in state change
            if (std::abs(gripper_command_ - gripper_state_) > 0.1)
            {
                float movement = (gripper_command_ - gripper_state_) * 0.5f;
                gripper_state_ += movement;
            }
            else
            {
                gripper_state_ = gripper_command_;
            }
            return hardware_interface::return_type::OK;
        }
        
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Gripper not initialized");
            return hardware_interface::return_type::ERROR;
        }

        // For VG gripper, we don't have a direct "position" reading
        // We can check vacuum levels to determine if we're gripping
        try
        {
            // Read vacuum levels to determine state
            float vacuum_a = gripper_->getChannelAVacuum();
            float vacuum_b = gripper_->getChannelBVacuum();
            
            // If either channel has significant vacuum, consider it gripped
            // Threshold can be adjusted based on application
            float vacuum_threshold = 10.0f; // 10% vacuum threshold
            
            bool is_gripping = (vacuum_a > vacuum_threshold) || (vacuum_b > vacuum_threshold);
            
            // Update state based on vacuum reading
            if (is_gripping)
            {
                gripper_state_ = 1.0;
            }
            else
            {
                gripper_state_ = 0.0;
            }
            
            RCLCPP_DEBUG(rclcpp::get_logger("VGHardwareInterface"), 
                        "VG state - Vacuum A: %.1f%%, Vacuum B: %.1f%%, State: %.1f", 
                        vacuum_a, vacuum_b, gripper_state_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Failed to read gripper state: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VGHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        
        if (use_fake_hardware_)
        {
            // For fake hardware, just accept the command
            return hardware_interface::return_type::OK;
        }
        
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Gripper not initialized");
            return hardware_interface::return_type::ERROR;
        }
        
        // Only send command if it has changed significantly
        // Use threshold to avoid noise
        if (std::abs(gripper_command_ - gripper_state_) > 0.5)
        {
            try
            {
                if (gripper_command_ > 0.5)
                {
                    // Command to grip
                    gripper_->grip();
                    RCLCPP_DEBUG(rclcpp::get_logger("VGHardwareInterface"), "Commanded VG to GRIP");
                }
                else
                {
                    // Command to release
                    gripper_->release();
                    RCLCPP_DEBUG(rclcpp::get_logger("VGHardwareInterface"), "Commanded VG to RELEASE");
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("VGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::VGHardwareInterface, hardware_interface::ActuatorInterface)
