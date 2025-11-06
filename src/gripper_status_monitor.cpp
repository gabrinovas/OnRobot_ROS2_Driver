#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class GripperStatusMonitor : public rclcpp::Node {
public:
    GripperStatusMonitor() : Node("gripper_status_monitor") {
        // Get parameters
        this->declare_parameter("onrobot_type", "rg2");
        onrobot_type_ = this->get_parameter("onrobot_type").as_string();

        // Create status publisher
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("gripper_status", 10);

        // Start monitoring timer
        monitor_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&GripperStatusMonitor::monitor_status, this));

        RCLCPP_INFO(this->get_logger(), "Gripper Status Monitor started for %s", onrobot_type_.c_str());
    }

private:
    void monitor_status() {
        auto status_msg = std_msgs::msg::String();
        
        if (onrobot_type_.find("2fg") != std::string::npos) {
            // 2FG series status monitoring
            status_msg.data = "2FG " + onrobot_type_ + " status: Monitoring active";
        } else {
            // RG series status monitoring
            status_msg.data = "RG " + onrobot_type_ + " status: Monitoring active";
        }
        
        status_publisher_->publish(status_msg);
        RCLCPP_DEBUG(this->get_logger(), status_msg.data.c_str());
    }

    std::string onrobot_type_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperStatusMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}