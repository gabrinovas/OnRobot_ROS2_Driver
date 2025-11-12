#include "onrobot_driver/fg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver
{

FGHardwareInterface::FGHardwareInterface()
    : finger_width_state_(0.0),
      finger_width_command_(0.0),
      device_address_(65),
      use_fake_hardware_(false)
{
}

FGHardwareInterface::~FGHardwareInterface() = default;

hardware_interface::CallbackReturn FGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    info_ = info;

    // === 1. Obtener tipo de gripper ===
    if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Missing 'onrobot_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    onrobot_type_ = info.hardware_parameters.at("onrobot_type");

    // === 2. Validar tipo soportado: 2fg7, 2fg14, 3fg15 ===
    if (onrobot_type_ != "2fg7" && onrobot_type_ != "2fg14" && onrobot_type_ != "3fg15")
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"),
                     "Invalid onrobot_type: '%s'. Supported: 2fg7, 2fg14, 3fg15", onrobot_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 3. Tipo de conexión ===
    if (info.hardware_parameters.find("connection_type") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Missing 'connection_type' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    connection_type_ = info.hardware_parameters.at("connection_type");

    // === 4. Parámetros de conexión ===
    if (connection_type_ == "tcp")
    {
        if (info.hardware_parameters.find("ip_address") == info.hardware_parameters.end() ||
            info.hardware_parameters.find("port") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Missing ip_address or port for TCP");
            return hardware_interface::CallbackReturn::ERROR;
        }
        ip_address_ = info.hardware_parameters.at("ip_address");
        port_ = std::stoi(info.hardware_parameters.at("port"));
    }
    else if (connection_type_ == "serial")
    {
        if (info.hardware_parameters.find("device") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Missing 'device' for serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        device_ = info.hardware_parameters.at("device");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 5. Dirección del dispositivo ===
    if (info.hardware_parameters.find("device_address") != info.hardware_parameters.end())
    {
        device_address_ = std::stoi(info.hardware_parameters.at("device_address"));
    }

    // === 6. Modo fake ===
    if (info.hardware_parameters.find("use_fake_hardware") != info.hardware_parameters.end())
    {
        use_fake_hardware_ = (info.hardware_parameters.at("use_fake_hardware") == "true");
    }

    // === 7. Prefijo ===
    if (info.hardware_parameters.find("prefix") == info.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Missing 'prefix' parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }
    prefix_ = info.hardware_parameters.at("prefix");

    // === 8. Validar joint ===
    if (info.joints.size() != 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Expected 1 joint, got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info.joints[0].name != "finger_width")
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Expected joint 'finger_width', got '%s'",
                     info.joints[0].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // === 9. Inicialización según tipo ===
    double initial_width = 0.0;
    if (onrobot_type_ == "3fg15")
        initial_width = 0.075;  // Mitad del rango: 0.15 m
    else if (onrobot_type_ == "2fg14")
        initial_width = 0.070;  // Mitad: 0.14 m
    else
        initial_width = 0.035;  // 2fg7: 0.07 m

    finger_width_state_ = initial_width;
    finger_width_command_ = initial_width;

    RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"),
                "OnRobot %s Hardware Interface initialized (fake: %s, prefix: '%s')",
                onrobot_type_.c_str(), use_fake_hardware_ ? "true" : "false", prefix_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    if (use_fake_hardware_)
    {
        RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"), "Using fake hardware for %s", onrobot_type_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    try
    {
        if (connection_type_ == "tcp")
        {
            RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"),
                        "Connecting via TCP to %s:%d (address: %d)", ip_address_.c_str(), port_, device_address_);
            gripper_ = std::make_unique<FG>(onrobot_type_, ip_address_, port_, device_address_);
        }
        else if (connection_type_ == "serial")
        {
            RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"),
                        "Connecting via Serial to %s (address: %d)", device_.c_str(), device_address_);
            gripper_ = std::make_unique<FG>(onrobot_type_, device_, device_address_);
        }

        // Leer ancho inicial
        float width = gripper_->getWidth();
        if (width < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Failed to read initial width");
            return hardware_interface::CallbackReturn::ERROR;
        }

        finger_width_state_ = width;
        finger_width_command_ = width;

        RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"),
                    "%s configured. Initial diameter: %.3f m", onrobot_type_.c_str(), width);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Failed to configure gripper: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"), "%s Hardware Interface cleaned up", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"), "%s activated", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"), "%s deactivated", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
    gripper_.reset();
    RCLCPP_INFO(rclcpp::get_logger("FGHardwareInterface"), "%s shutdown", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "%s error state", onrobot_type_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FGHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "velocity", &finger_width_state_)); // Dummy velocity
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FGHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "position", &finger_width_command_));
    return command_interfaces;
}


hardware_interface::return_type FGHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
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
        float width = gripper_->getWidth();
        if (width >= 0.0f) finger_width_state_ = width;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Read error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FGHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);

    if (use_fake_hardware_) return hardware_interface::return_type::OK;
    if (!gripper_) return hardware_interface::return_type::ERROR;

    // === Validar rango según tipo ===
    double max_width = (onrobot_type_ == "3fg15") ? 0.150 :
                       (onrobot_type_ == "2fg14") ? 0.140 : 0.070;

    if (finger_width_command_ < 0.0 || finger_width_command_ > max_width)
    {
        RCLCPP_WARN(rclcpp::get_logger("FGHardwareInterface"),
                    "Command %.3f m out of range [0.0, %.3f] for %s",
                    finger_width_command_, max_width, onrobot_type_.c_str());
        return hardware_interface::return_type::OK;
    }

    if (std::abs(finger_width_command_ - finger_width_state_) > 0.001)
    {
        try
        {
            gripper_->moveGripper(finger_width_command_);
            RCLCPP_DEBUG(rclcpp::get_logger("FGHardwareInterface"),
                         "Sent command: %.3f m (%s)", finger_width_command_, onrobot_type_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FGHardwareInterface"), "Write error: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::FGHardwareInterface, hardware_interface::ActuatorInterface)