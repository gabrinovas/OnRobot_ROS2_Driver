#include "rclcpp/rclcpp.hpp"
#include "onrobot_driver/RG.hpp"

class OnRobotNode : public rclcpp::Node {
public:
    OnRobotNode() : Node("onrobot_driver_node") {
        try {
            // Testing: create an RG object using Serial connection.
            onrobot_ = std::make_unique<RG>("rg2", "/tmp/ttyUR");
            RCLCPP_INFO(this->get_logger(), "OnRobot gripper connected successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create RG instance: %s", e.what());
        }

        // Set up publishers, subscribers, or services to control the gripper.
        
    }

private:
    std::unique_ptr<RG> onrobot_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OnRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
