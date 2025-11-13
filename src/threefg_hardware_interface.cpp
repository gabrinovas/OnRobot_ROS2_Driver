#include "onrobot_driver/threefg_hardware_interface.hpp"
#include "onrobot_driver/GripperDetection.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

    ThreeFGHardwareInterface::ThreeFGHardwareInterface()
        : finger_width_state_(0.0),
          finger_width_command_(0.0),
          device_address_(65),
          use_fake_hardware_(false),
          auto_detect_(false)
    {
    }

    ThreeFGHardwareInterface::~ThreeFGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        info_ = info;
        
        if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end())
        {
            onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (onrobot_type_ != "3fg15" && onrobot_type_ != "3fg25")
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Invalid onrobot_type for 3FG series: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Connection parameters
        if (info.hardware_parameters.find("connection_type") != info.hardware_parameters.end())
        {
            connection_type_ = info.hardware_parameters.at("connection_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing connection_type parameter");
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
                RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing ip_address or port for TCP connection");
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
                RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
        {
            device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
        } else {
            // Default device address for 3FG series is 65
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
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.joints.size() != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Expected exactly 1 joint, got %zu", info.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.joints[0].name != "finger_width") {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Expected joint name 'finger_width', got '%s'", info.joints[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize joint variables based on gripper type
        if (onrobot_type_ == "3fg15") {
            finger_width_state_ = 0.075; // Start at half-open for 3FG15
        } else {
            finger_width_state_ = 0.125; // Start at half-open for 3FG25
        }
        finger_width_command_ = finger_width_state_;
        
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                   "3FG Hardware Interface initialized for %s (auto_detect: %s, fake hardware: %s)", 
                   onrobot_type_.c_str(), 
                   auto_detect_ ? "true" : "false",
                   use_fake_hardware_ ? "true" : "false");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        if (use_fake_hardware_)
        {
            RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "Using fake hardware for 3FG gripper");
            if (onrobot_type_ == "3fg15") {
                finger_width_state_ = 0.075;
            } else {
                finger_width_state_ = 0.125;
            }
            finger_width_command_ = finger_width_state_;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

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
                RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                            "Unsupported connection type: %s", connection_type_.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Auto-detect gripper type if enabled
            if (auto_detect_) {
                std::string detected_type = GripperDetection::detectGripperType(temp_connection, device_address_);
                
                if (!detected_type.empty()) {
                    if (detected_type != onrobot_type_) {
                        RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                                   "Auto-detected gripper type: %s (configured: %s)", 
                                   detected_type.c_str(), onrobot_type_.c_str());
                        
                        // Check if detected type is compatible with this hardware interface
                        if (detected_type.find("3fg") == std::string::npos) {
                            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                                       "Detected gripper type %s is not a 3FG series gripper. "
                                       "This hardware interface only supports 3FG series.",
                                       detected_type.c_str());
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                        
                        onrobot_type_ = detected_type;
                    } else {
                        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                                   "Auto-detection confirmed configured type: %s", onrobot_type_.c_str());
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                               "Auto-detection failed. Using configured type: %s", onrobot_type_.c_str());
                }
            }

            // Now create the actual gripper instance
            if (connection_type_ == "tcp")
            {
                RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                           "Creating TCP connection to %s:%d, device address: %d", 
                           ip_address_.c_str(), port_, device_address_);
                gripper_ = std::make_unique<ThreeFG>(onrobot_type_, ip_address_, port_, device_address_);
            }
            else if (connection_type_ == "serial")
            {
                RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                           "Creating Serial connection to %s, device address: %d", 
                           device_.c_str(), device_address_);
                gripper_ = std::make_unique<ThreeFG>(onrobot_type_, device_, device_address_);
            }

            float initial_width = gripper_->getWidth();
            if (initial_width < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                            "Failed to read initial gripper width");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            finger_width_state_ = initial_width;
            finger_width_command_ = initial_width;
            
            RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                       "3FG gripper configured. Type: %s, Initial width: %.3f m", 
                       onrobot_type_.c_str(), finger_width_state_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                        "Failed to create ThreeFG instance: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG Hardware Interface cleaned up");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG Hardware Interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG Hardware Interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
        }
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG Hardware Interface shutdown");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "3FG Hardware Interface error occurred");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ThreeFGHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "velocity", &finger_width_state_));
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
            float movement = (finger_width_command_ - finger_width_state_) * 0.1f;
            finger_width_state_ += movement;
            return hardware_interface::return_type::OK;
        }
        
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Gripper not initialized");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            float new_width = gripper_->getWidth();
            if (new_width >= 0.0) {
                finger_width_state_ = new_width;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to read gripper width");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to read gripper state: %s", e.what());
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
            RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Gripper not initialized");
            return hardware_interface::return_type::ERROR;
        }
        
        if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
        {
            try
            {
                // Check if gripper is ready before sending command
                if (gripper_->waitUntilReady(100)) { // 100ms timeout
                    gripper_->moveGripper(finger_width_command_);
                    RCLCPP_DEBUG(rclcpp::get_logger("ThreeFGHardwareInterface"), "Commanded gripper to width: %.3f m", finger_width_command_);
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                              "Gripper busy, skipping command");
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::ThreeFGHardwareInterface, hardware_interface::ActuatorInterface)
