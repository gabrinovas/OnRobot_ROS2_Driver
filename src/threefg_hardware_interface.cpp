#include "onrobot_driver/threefg/threefg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace onrobot_driver
{

ThreeFGHardwareInterface::ThreeFGHardwareInterface()
{
    onrobot_type_ = "3fg15";
    min_width_ = 0.0;
    max_width_ = 0.150;
    max_force_ = 140.0;
    finger_width_state_ = 0.075;
    finger_width_command_ = 0.075;
    finger_width_effort_command_ = 70.0;
}

ThreeFGHardwareInterface::~ThreeFGHardwareInterface() = default;

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (info.hardware_parameters.find("onrobot_type") == info.hardware_parameters.end() ||
        info.hardware_parameters.at("onrobot_type") != "3fg15")
    {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), "Expected 'onrobot_type' parameter to be '3fg15'");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return OnRobotHardwareInterfaceBase::on_init(info);
}

bool ThreeFGHardwareInterface::instantiateGripper()
{
    if (connection_type_ == "tcp")
    {
        gripper_ = std::make_unique<ThreeFG>(ip_address_, port_, device_address_, []() { return rclcpp::ok(); });
    }
    else if (connection_type_ == "serial")
    {
        gripper_ = std::make_unique<ThreeFG>(device_, device_address_, []() { return rclcpp::ok(); });
    }
    return gripper_ != nullptr;
}

void ThreeFGHardwareInterface::destroyGripper()
{
    gripper_.reset();
}

OnRobotGripperBase *ThreeFGHardwareInterface::getGripperBase()
{
    return gripper_.get();
}

} // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(onrobot_driver::ThreeFGHardwareInterface, hardware_interface::ActuatorInterface)