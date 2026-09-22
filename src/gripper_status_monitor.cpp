#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "onrobot_driver/common/IModbusConnection.hpp"
#include "onrobot_driver/common/TCPConnectionWrapper.hpp"
#include "onrobot_driver/common/SerialConnectionWrapper.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusUtils.hpp"

class GripperStatusMonitor : public rclcpp::Node
{
public:
    GripperStatusMonitor()
        : Node("gripper_status_monitor"),
          updater_(this),
          current_width_(0.0),
          current_velocity_(0.0),
          current_effort_(0.0),
          has_joint_state_(false)
    {
        // Parameters
        this->declare_parameter("onrobot_type", "2fg7");
        this->declare_parameter("connection_type", "tcp");
        this->declare_parameter("ip_address", "192.168.1.1");
        this->declare_parameter("port", 502);
        this->declare_parameter("device", "/tmp/ttyUR");
        this->declare_parameter("compute_box_address", 63);
        this->declare_parameter("use_fake_hardware", false);

        onrobot_type_ = this->get_parameter("onrobot_type").as_string();
        connection_type_ = this->get_parameter("connection_type").as_string();
        ip_address_ = this->get_parameter("ip_address").as_string();
        port_ = this->get_parameter("port").as_int();
        device_ = this->get_parameter("device").as_string();
        compute_box_address_ = this->get_parameter("compute_box_address").as_int();
        use_fake_hardware_ = this->get_parameter("use_fake_hardware").as_bool();

        // Diagnostics setup
        updater_.setHardwareID("OnRobot " + onrobot_type_);
        updater_.add("Gripper Telemetry", this, &GripperStatusMonitor::produceDiagnostics);

        // Legacy string status publisher
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("gripper_status", 10);

        // Subscribe to joint_states to track telemetry
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&GripperStatusMonitor::jointStateCallback, this, std::placeholders::_1));

        // Power reset service for safety switch recovery (Compute Box Device 63, Reg 0)
        reset_power_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_power",
            std::bind(&GripperStatusMonitor::handleResetPower, this, std::placeholders::_1, std::placeholders::_2));

        // Timer for diagnostics and legacy status publishing
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GripperStatusMonitor::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "Gripper Status Monitor started for %s with diagnostic_updater and reset_power service",
                    onrobot_type_.c_str());
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i].find("finger_width") != std::string::npos)
            {
                if (i < msg->position.size())
                {
                    current_width_ = msg->position[i];
                }
                if (i < msg->velocity.size())
                {
                    current_velocity_ = msg->velocity[i];
                }
                if (i < msg->effort.size())
                {
                    current_effort_ = msg->effort[i];
                }
                last_msg_time_ = this->now();
                has_joint_state_ = true;
                break;
            }
        }
    }

    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        auto now = this->now();
        double time_since_last_msg = (has_joint_state_) ? (now - last_msg_time_).seconds() : 999.0;

        if (!has_joint_state_)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for initial joint states");
        }
        else if (time_since_last_msg > 3.0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Communication timeout (> 3s without telemetry)");
        }
        else if (time_since_last_msg > 1.0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Delayed telemetry update");
        }
        else
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Gripper operational");
        }

        stat.add("Gripper Model", onrobot_type_);
        stat.add("Connection Type", connection_type_);
        stat.addf("Width (mm)", "%.2f", current_width_ * 1000.0);
        stat.addf("Velocity (mm/s)", "%.2f", current_velocity_ * 1000.0);
        stat.addf("Effort (N)", "%.1f", current_effort_);
        stat.addf("Seconds since last update", "%.2f", time_since_last_msg);
    }

    void handleResetPower(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (use_fake_hardware_)
        {
            RCLCPP_INFO(this->get_logger(), "Simulated tool power reset executed (fake hardware mode).");
            res->success = true;
            res->message = "Fake hardware: Simulated tool power reset command executed successfully.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executing tool power reset via Compute Box (address %d)...", compute_box_address_);
        try
        {
            std::unique_ptr<IModbusConnection> conn;
            if (connection_type_ == "tcp")
            {
                conn = std::make_unique<TCPConnectionWrapper>(ip_address_, port_);
            }
            else
            {
                conn = std::make_unique<SerialConnectionWrapper>(device_);
            }

            // Write 2 to register 0 (0x0000) of Device 63 per OnRobot Connectivity Guide
            std::vector<MB::ModbusCell> values = {MB::ModbusCell(static_cast<uint16_t>(2))};
            MB::ModbusRequest req(compute_box_address_, MB::utils::WriteSingleAnalogOutputRegister, 0, 1, values);
            conn->sendRequest(req);
            conn->close();

            res->success = true;
            res->message = "Tool power reset command sent successfully to Compute Box.";
            RCLCPP_INFO(this->get_logger(), "Tool power reset command executed successfully.");
        }
        catch (const std::exception &e)
        {
            res->success = false;
            res->message = std::string("Failed to reset tool power: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Error resetting tool power: %s", e.what());
        }
    }

    void onTimer()
    {
        updater_.force_update();

        auto status_msg = std_msgs::msg::String();
        if (has_joint_state_)
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "OnRobot %s: width=%.1fmm, effort=%.1fN",
                     onrobot_type_.c_str(), current_width_ * 1000.0, current_effort_);
            status_msg.data = buf;
        }
        else
        {
            status_msg.data = "OnRobot " + onrobot_type_ + ": Initializing...";
        }
        status_publisher_->publish(status_msg);
    }

    std::string onrobot_type_;
    std::string connection_type_;
    std::string ip_address_;
    int port_;
    std::string device_;
    int compute_box_address_;
    bool use_fake_hardware_;

    double current_width_;
    double current_velocity_;
    double current_effort_;
    bool has_joint_state_;
    rclcpp::Time last_msg_time_;

    diagnostic_updater::Updater updater_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_power_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperStatusMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}