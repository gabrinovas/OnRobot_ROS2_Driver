#pragma once

#include <memory>
#include <string>

#include "onrobot_driver/common/OnRobotHardwareInterfaceBase.hpp"
#include "onrobot_driver/threefg/ThreeFG.hpp"

namespace onrobot_driver
{

class ThreeFGHardwareInterface : public OnRobotHardwareInterfaceBase
{
public:
    ThreeFGHardwareInterface();
    ~ThreeFGHardwareInterface() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

protected:
    bool instantiateGripper() override;
    void destroyGripper() override;
    OnRobotGripperBase *getGripperBase() override;

private:
    std::unique_ptr<ThreeFG> gripper_;
};

} // namespace onrobot_driver