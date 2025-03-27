#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "onrobot_driver/RG.hpp"
using std::placeholders::_1;

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
        
        // Get width of the gripper and print it.
        float width = onrobot_->getWidthWithOffset();
        if (width >= 0) {
            RCLCPP_INFO(this->get_logger(), "Width: %.1f mm", width);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read width.");
        }

        // Set up bool topic to control the gripper.
        toggle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "gripper_toggle", 10, std::bind(&OnRobotNode::gripperToggleCallback, this, _1));

        // Set up float topic to control the gripper width
        move_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "gripper_move", 10, std::bind(&OnRobotNode::gripperMoveCallback, this, _1));

    }

private:
    std::unique_ptr<RG> onrobot_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr move_sub_;

    void gripperToggleCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            onrobot_->closeGripper(400);
        } else {
            onrobot_->openGripper(400);
        }
    }

    void gripperMoveCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        onrobot_->moveGripper(msg->data);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OnRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
