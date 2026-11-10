#pragma once

#include <memory>
#include <string>

#include "onrobot_driver/common/OnRobotHardwareInterfaceBase.hpp"
#include "onrobot_driver/twofg/TwoFG.hpp"

namespace onrobot_driver
{

class TwoFGHardwareInterface : public OnRobotHardwareInterfaceBase
{
public:
    TwoFGHardwareInterface();
    ~TwoFGHardwareInterface() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

protected:
    bool instantiateGripper() override;
    void destroyGripper() override;
    OnRobotGripperBase *getGripperBase() override;

private:
    std::unique_ptr<TwoFG> gripper_;
};

} // namespace onrobot_driver