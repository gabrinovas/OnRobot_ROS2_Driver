#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <onrobot_driver/TwoFG.hpp>
#include <onrobot_driver/ThreeFG.hpp>
#include <onrobot_driver/VG.hpp>
#include <onrobot_driver/RG.hpp>
#include <onrobot_driver/GripperDetection.hpp>
#include <onrobot_driver/TCPConnectionWrapper.hpp>
#include <onrobot_driver/SerialConnectionWrapper.hpp>
#include <memory>
#include <chrono>

class GripperStatusMonitor : public rclcpp::Node {
public:
    GripperStatusMonitor() : Node("gripper_status_monitor") {
        // Get parameters
        this->declare_parameter("onrobot_type", "rg2");
        this->declare_parameter("connection_type", "tcp");
        this->declare_parameter("ip_address", "192.168.1.1");
        this->declare_parameter("port", 502);
        this->declare_parameter("device", "/tmp/ttyUR");
        this->declare_parameter("device_address", 65);
        this->declare_parameter("prefix", "");
        this->declare_parameter("monitor_rate", 2.0);
        this->declare_parameter("auto_detect", false);

        onrobot_type_ = this->get_parameter("onrobot_type").as_string();
        connection_type_ = this->get_parameter("connection_type").as_string();
        ip_address_ = this->get_parameter("ip_address").as_string();
        port_ = this->get_parameter("port").as_int();
        device_ = this->get_parameter("device").as_string();
        device_address_ = this->get_parameter("device_address").as_int();
        prefix_ = this->get_parameter("prefix").as_string();
        double monitor_rate = this->get_parameter("monitor_rate").as_double();
        auto_detect_ = this->get_parameter("auto_detect").as_bool();

        // Create status publishers
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("gripper_status", 10);
        detailed_status_publisher_ = this->create_publisher<std_msgs::msg::String>("gripper_detailed_status", 10);
        detection_publisher_ = this->create_publisher<std_msgs::msg::String>("gripper_detection", 10);

        // Initialize gripper based on type
        if (!initialize_gripper()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper. Status monitoring will use fake data.");
            use_fake_data_ = true;
        } else {
            use_fake_data_ = false;
            RCLCPP_INFO(this->get_logger(), "Successfully initialized %s gripper", onrobot_type_.c_str());
        }

        // Start monitoring timer
        auto monitor_period = std::chrono::duration<double>(1.0 / monitor_rate);
        monitor_timer_ = this->create_wall_timer(
            monitor_period,
            std::bind(&GripperStatusMonitor::monitor_status, this));

        RCLCPP_INFO(this->get_logger(), 
                   "Gripper Status Monitor started for %s (auto_detect: %s)", 
                   onrobot_type_.c_str(), 
                   auto_detect_ ? "true" : "false");
    }

    ~GripperStatusMonitor() {
        if (twofg_gripper_) {
            twofg_gripper_.reset();
        }
        if (threefg_gripper_) {
            threefg_gripper_.reset();
        }
        if (vg_gripper_) {
            vg_gripper_.reset();
        }
        if (rg_gripper_) {
            rg_gripper_.reset();
        }
        if (temp_connection_) {
            temp_connection_->close();
        }
    }

private:
    bool initialize_gripper() {
        try {
            // First, try to auto-detect if enabled
            if (auto_detect_) {
                std::string detected_type = perform_auto_detection();
                if (!detected_type.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Auto-detected gripper type: %s", detected_type.c_str());
                    onrobot_type_ = detected_type;
                    
                    // Publish detection result
                    auto detection_msg = std_msgs::msg::String();
                    detection_msg.data = "Auto-detected gripper type: " + detected_type;
                    detection_publisher_->publish(detection_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Auto-detection failed. Using configured type: %s", onrobot_type_.c_str());
                }
            }

            // Now initialize the appropriate gripper
            if (onrobot_type_.find("2fg") != std::string::npos) {
                // Initialize 2FG gripper
                if (connection_type_ == "tcp") {
                    twofg_gripper_ = std::make_unique<TwoFG>(onrobot_type_, ip_address_, port_, device_address_);
                } else {
                    twofg_gripper_ = std::make_unique<TwoFG>(onrobot_type_, device_, device_address_);
                }
                RCLCPP_INFO(this->get_logger(), "2FG gripper initialized successfully: %s", onrobot_type_.c_str());
                return true;
            } 
            else if (onrobot_type_.find("3fg") != std::string::npos) {
                // Initialize 3FG gripper
                if (connection_type_ == "tcp") {
                    threefg_gripper_ = std::make_unique<ThreeFG>(onrobot_type_, ip_address_, port_, device_address_);
                } else {
                    threefg_gripper_ = std::make_unique<ThreeFG>(onrobot_type_, device_, device_address_);
                }
                RCLCPP_INFO(this->get_logger(), "3FG gripper initialized successfully: %s", onrobot_type_.c_str());
                return true;
            }
            else if (onrobot_type_.find("vg") != std::string::npos) {
                // Initialize VG gripper
                if (connection_type_ == "tcp") {
                    vg_gripper_ = std::make_unique<VG>(onrobot_type_, ip_address_, port_, device_address_);
                } else {
                    vg_gripper_ = std::make_unique<VG>(onrobot_type_, device_, device_address_);
                }
                RCLCPP_INFO(this->get_logger(), "VG gripper initialized successfully: %s", onrobot_type_.c_str());
                return true;
            }
            else if (onrobot_type_.find("rg") != std::string::npos) {
                // Initialize RG gripper
                if (connection_type_ == "tcp") {
                    rg_gripper_ = std::make_unique<RG>(onrobot_type_, ip_address_, port_);
                } else {
                    rg_gripper_ = std::make_unique<RG>(onrobot_type_, device_);
                }
                RCLCPP_INFO(this->get_logger(), "RG gripper initialized successfully: %s", onrobot_type_.c_str());
                return true;
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Unsupported gripper type: %s. Using fake data.", onrobot_type_.c_str());
                return false;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper: %s", e.what());
            return false;
        }
    }

    std::string perform_auto_detection() {
        try {
            // Create temporary connection for detection
            if (connection_type_ == "tcp") {
                temp_connection_ = std::make_unique<TCPConnectionWrapper>(ip_address_, port_);
            } else if (connection_type_ == "serial") {
                temp_connection_ = std::make_unique<SerialConnectionWrapper>(device_);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported connection type for auto-detection: %s", connection_type_.c_str());
                return "";
            }

            // Perform detection
            std::string detected_type = GripperDetection::detectGripperType(temp_connection_, device_address_);
            
            if (!detected_type.empty()) {
                RCLCPP_INFO(this->get_logger(), "Successfully detected gripper: %s", detected_type.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Auto-detection returned empty result");
            }
            
            return detected_type;
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during auto-detection: %s", e.what());
            return "";
        }
    }

    void monitor_status() {
        auto status_msg = std_msgs::msg::String();
        auto detailed_msg = std_msgs::msg::String();
        
        if (use_fake_data_) {
            monitor_fake_status(status_msg, detailed_msg);
        } else {
            monitor_real_status(status_msg, detailed_msg);
        }
        
        status_publisher_->publish(status_msg);
        detailed_status_publisher_->publish(detailed_msg);
        
        RCLCPP_DEBUG(this->get_logger(), status_msg.data.c_str());
    }

    void monitor_real_status(std_msgs::msg::String& status_msg, std_msgs::msg::String& detailed_msg) {
        try {
            if (twofg_gripper_) {
                monitor_2fg_status(status_msg, detailed_msg);
            } 
            else if (threefg_gripper_) {
                monitor_3fg_status(status_msg, detailed_msg);
            }
            else if (vg_gripper_) {
                monitor_vg_status(status_msg, detailed_msg);
            }
            else if (rg_gripper_) {
                monitor_rg_status(status_msg, detailed_msg);
            }
            else {
                status_msg.data = "No gripper initialized";
                detailed_msg.data = "No gripper connection available";
            }
        } catch (const std::exception &e) {
            status_msg.data = "Error reading gripper status: " + std::string(e.what());
            detailed_msg.data = "Communication error with gripper";
            RCLCPP_ERROR(this->get_logger(), "Error monitoring gripper status: %s", e.what());
        }
    }

    void monitor_2fg_status(std_msgs::msg::String& status_msg, std_msgs::msg::String& detailed_msg) {
        auto status = twofg_gripper_->getStatus();
        float width = twofg_gripper_->getWidth();
        
        std::string status_str = "2FG " + onrobot_type_ + " - ";
        std::string detailed_str = "2FG " + onrobot_type_ + " Detailed Status:\n";
        
        // Basic status
        if (status[0]) {
            status_str += "BUSY, ";
        } else {
            status_str += "READY, ";
        }
        
        if (status[1]) {
            status_str += "GRIP_DETECTED, ";
        }
        
        status_str += "Width: " + std::to_string(width * 1000) + "mm";
        
        // Detailed status
        detailed_str += "Gripper Type: " + onrobot_type_ + "\n";
        detailed_str += "Width: " + std::to_string(width * 1000) + " mm\n";
        detailed_str += "Busy: " + std::string(status[0] ? "YES" : "NO") + "\n";
        detailed_str += "Grip Detected: " + std::string(status[1] ? "YES" : "NO") + "\n";
        detailed_str += "Calibration Error: " + std::string(status[2] ? "YES" : "NO") + "\n";
        detailed_str += "Sensor Error: " + std::string(status[3] ? "YES" : "NO") + "\n";
        
        // Add error information
        if (status[2] || status[3]) {
            detailed_str += "ALERT: Gripper needs attention!\n";
            if (status[2]) detailed_str += "  - Calibration required\n";
            if (status[3]) detailed_str += "  - Linear sensor error\n";
        }
        
        status_msg.data = status_str;
        detailed_msg.data = detailed_str;
    }

    void monitor_3fg_status(std_msgs::msg::String& status_msg, std_msgs::msg::String& detailed_msg) {
        auto status = threefg_gripper_->getStatus();
        float width = threefg_gripper_->getWidth();
        
        std::string status_str = "3FG " + onrobot_type_ + " - ";
        std::string detailed_str = "3FG " + onrobot_type_ + " Detailed Status:\n";
        
        // Basic status
        if (status[0]) {
            status_str += "BUSY, ";
        } else {
            status_str += "READY, ";
        }
        
        if (status[1]) {
            status_str += "GRIP_DETECTED, ";
        }
        
        if (status[2]) {
            status_str += "FORCE_GRIP, ";
        }
        
        status_str += "Diameter: " + std::to_string(width * 1000) + "mm";
        
        // Detailed status
        detailed_str += "Gripper Type: " + onrobot_type_ + "\n";
        detailed_str += "Diameter: " + std::to_string(width * 1000) + " mm\n";
        detailed_str += "Busy: " + std::string(status[0] ? "YES" : "NO") + "\n";
        detailed_str += "Grip Detected: " + std::string(status[1] ? "YES" : "NO") + "\n";
        detailed_str += "Force Grip Detected: " + std::string(status[2] ? "YES" : "NO") + "\n";
        detailed_str += "Calibration OK: " + std::string(status[3] ? "YES" : "NO") + "\n";
        
        // Add status interpretation
        if (status[0]) {
            detailed_str += "Status: Gripper is moving\n";
        } else if (status[1]) {
            detailed_str += "Status: Object gripped\n";
        } else {
            detailed_str += "Status: Ready\n";
        }
        
        status_msg.data = status_str;
        detailed_msg.data = detailed_str;
    }

    void monitor_vg_status(std_msgs::msg::String& status_msg, std_msgs::msg::String& detailed_msg) {
        float vacuum_a = vg_gripper_->getChannelAVacuum();
        float vacuum_b = vg_gripper_->getChannelBVacuum();
        auto status = vg_gripper_->getStatus();
        bool is_gripping = vg_gripper_->isGripping();
        
        std::string status_str = "VG " + onrobot_type_ + " - ";
        std::string detailed_str = "VG " + onrobot_type_ + " Detailed Status:\n";
        
        // Basic status
        if (is_gripping) {
            status_str += "GRIPPING, ";
        } else {
            status_str += "RELEASED, ";
        }
        
        status_str += "Vacuum A: " + std::to_string(vacuum_a) + "%, ";
        status_str += "Vacuum B: " + std::to_string(vacuum_b) + "%";
        
        // Detailed status
        detailed_str += "Gripper Type: " + onrobot_type_ + "\n";
        detailed_str += "Channel A Vacuum: " + std::to_string(vacuum_a) + " %\n";
        detailed_str += "Channel B Vacuum: " + std::to_string(vacuum_b) + " %\n";
        detailed_str += "Gripping: " + std::string(is_gripping ? "YES" : "NO") + "\n";
        detailed_str += "Channel A Active: " + std::string(status[0] ? "YES" : "NO") + "\n";
        detailed_str += "Channel B Active: " + std::string(status[1] ? "YES" : "NO") + "\n";
        
        // Add grip quality information
        if (is_gripping) {
            float max_vacuum = std::max(vacuum_a, vacuum_b);
            if (max_vacuum > 50.0f) {
                detailed_str += "Grip Quality: EXCELLENT\n";
            } else if (max_vacuum > 20.0f) {
                detailed_str += "Grip Quality: GOOD\n";
            } else {
                detailed_str += "Grip Quality: WEAK - Check workpiece\n";
            }
        } else {
            detailed_str += "Grip Quality: NOT GRIPPING\n";
        }
        
        // Add channel usage info
        if (status[0] && status[1]) {
            detailed_str += "Channels: Both active\n";
        } else if (status[0]) {
            detailed_str += "Channels: Only A active\n";
        } else if (status[1]) {
            detailed_str += "Channels: Only B active\n";
        } else {
            detailed_str += "Channels: None active\n";
        }
        
        status_msg.data = status_str;
        detailed_msg.data = detailed_str;
    }

    void monitor_rg_status(std_msgs::msg::String& status_msg, std_msgs::msg::String& detailed_msg) {
        auto status = rg_gripper_->getStatus();
        float width = rg_gripper_->getWidthWithOffset();
        
        std::string status_str = "RG " + onrobot_type_ + " - ";
        std::string detailed_str = "RG " + onrobot_type_ + " Detailed Status:\n";
        
        // Basic status
        if (status[0]) {
            status_str += "BUSY, ";
        } else {
            status_str += "READY, ";
        }
        
        if (status[1]) {
            status_str += "GRIP_DETECTED, ";
        }
        
        status_str += "Width: " + std::to_string(width * 1000) + "mm";
        
        // Detailed status
        detailed_str += "Gripper Type: " + onrobot_type_ + "\n";
        detailed_str += "Width: " + std::to_string(width * 1000) + " mm\n";
        detailed_str += "Busy: " + std::string(status[0] ? "YES" : "NO") + "\n";
        detailed_str += "Grip Detected: " + std::string(status[1] ? "YES" : "NO") + "\n";
        detailed_str += "Safety Switch 1: " + std::string(status[2] ? "PUSHED" : "OK") + "\n";
        detailed_str += "Safety Circuit 1: " + std::string(status[3] ? "ACTIVATED" : "OK") + "\n";
        detailed_str += "Safety Switch 2: " + std::string(status[4] ? "PUSHED" : "OK") + "\n";
        detailed_str += "Safety Circuit 2: " + std::string(status[5] ? "ACTIVATED" : "OK") + "\n";
        detailed_str += "Safety Error: " + std::string(status[6] ? "YES" : "NO") + "\n";
        
        // Add safety warnings
        if (status[3] || status[5]) {
            detailed_str += "ALERT: Safety circuits activated - gripper cannot move!\n";
        }
        if (status[6]) {
            detailed_str += "ALERT: Safety error detected!\n";
        }
        
        status_msg.data = status_str;
        detailed_msg.data = detailed_str;
    }

    void monitor_fake_status(std_msgs::msg::String& status_msg, std_msgs::msg::String& detailed_msg) {
        static int counter = 0;
        counter++;
        
        if (onrobot_type_.find("2fg") != std::string::npos) {
            status_msg.data = "2FG " + onrobot_type_ + " [FAKE] - READY, Width: 52.5mm";
            detailed_msg.data = "2FG " + onrobot_type_ + " Fake Detailed Status:\n" +
                               "Width: 52.5 mm\nBusy: NO\n" +
                               "Grip Detected: NO\nCalibration Error: NO\nSensor Error: NO\n" +
                               "Auto-detection: " + std::string(auto_detect_ ? "ENABLED" : "DISABLED") + "\n" +
                               "Cycle Count: " + std::to_string(counter);
        }
        else if (onrobot_type_.find("3fg") != std::string::npos) {
            status_msg.data = "3FG " + onrobot_type_ + " [FAKE] - READY, Diameter: 75.0mm";
            detailed_msg.data = "3FG " + onrobot_type_ + " Fake Detailed Status:\n" +
                               "Diameter: 75.0 mm\nBusy: NO\n" +
                               "Grip Detected: NO\nForce Grip Detected: NO\nCalibration OK: YES\n" +
                               "Auto-detection: " + std::string(auto_detect_ ? "ENABLED" : "DISABLED") + "\n" +
                               "Cycle Count: " + std::to_string(counter);
        }
        else if (onrobot_type_.find("vg") != std::string::npos) {
            status_msg.data = "VG " + onrobot_type_ + " [FAKE] - RELEASED, Vacuum A: 0.0%, Vacuum B: 0.0%";
            detailed_msg.data = "VG " + onrobot_type_ + " Fake Detailed Status:\n" +
                               "Channel A Vacuum: 0.0 %\nChannel B Vacuum: 0.0 %\n" +
                               "Gripping: NO\nChannel A Active: NO\nChannel B Active: NO\n" +
                               "Auto-detection: " + std::string(auto_detect_ ? "ENABLED" : "DISABLED") + "\n" +
                               "Cycle Count: " + std::to_string(counter);
        }
        else if (onrobot_type_.find("rg") != std::string::npos) {
            status_msg.data = "RG " + onrobot_type_ + " [FAKE] - READY, Width: 55.0mm";
            detailed_msg.data = "RG " + onrobot_type_ + " Fake Detailed Status:\n" +
                               "Width: 55.0 mm\nBusy: NO\n" +
                               "Grip Detected: NO\nSafety Circuits: OK\n" +
                               "Auto-detection: " + std::string(auto_detect_ ? "ENABLED" : "DISABLED") + "\n" +
                               "Cycle Count: " + std::to_string(counter);
        }
        else {
            status_msg.data = "Unknown gripper type: " + onrobot_type_ + " [FAKE]";
            detailed_msg.data = "Fake data for unknown gripper type\n" +
                               "Auto-detection: " + std::string(auto_detect_ ? "ENABLED" : "DISABLED") + "\n" +
                               "Cycle Count: " + std::to_string(counter);
        }
    }

    // Member variables
    std::string onrobot_type_;
    std::string connection_type_;
    std::string ip_address_;
    int port_;
    std::string device_;
    int device_address_;
    std::string prefix_;
    bool use_fake_data_;
    bool auto_detect_;

    // Gripper instances
    std::unique_ptr<TwoFG> twofg_gripper_;
    std::unique_ptr<ThreeFG> threefg_gripper_;
    std::unique_ptr<VG> vg_gripper_;
    std::unique_ptr<RG> rg_gripper_;
    
    // Temporary connection for auto-detection
    std::unique_ptr<IModbusConnection> temp_connection_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detailed_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_publisher_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperStatusMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}