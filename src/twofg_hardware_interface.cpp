#include "onrobot_driver/twofg/twofg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace onrobot_driver
{

TwoFGHardwareInterface::TwoFGHardwareInterface()
{
    onrobot_type_ = "2fg7";
    min_width_ = 0.0;
    max_width_ = 0.070;
    max_force_ = 70.0;
    finger_width_state_ = 0.035;
    finger_width_command_ = 0.035;
    finger_width_effort_command_ = 35.0;
}

TwoFGHardwareInterface::~TwoFGHardwareInterface() = default;

hardware_interface::CallbackReturn TwoFGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end() ||
        info.hardware_parameters.at("onrobot_type") != "2fg7")
    {
        RCLCPP_ERROR(rclcpp::get_logger("TwoFGHardwareInterface"), "Expected 'onrobot_type' parameter to be '2fg7'");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return OnRobotHardwareInterfaceBase::on_init(info);
}

bool TwoFGHardwareInterface::instantiateGripper()
{
    if (connection_type_ == "tcp")
    {
        gripper_ = std::make_unique<TwoFG>(onrobot_type_, ip_address_, port_, device_address_, []() { return rclcpp::ok(); });
    }
    else if (connection_type_ == "serial")
    {
        gripper_ = std::make_unique<TwoFG>(onrobot_type_, device_, device_address_, []() { return rclcpp::ok(); });
    }
    return gripper_ != nullptr;
}

void TwoFGHardwareInterface::destroyGripper()
{
    gripper_.reset();
}

OnRobotGripperBase *TwoFGHardwareInterface::getGripperBase()
{
    return gripper_.get();
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::TwoFGHardwareInterface, hardware_interface::ActuatorInterface)