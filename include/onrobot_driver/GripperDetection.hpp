#pragma once
#include <string>
#include <memory>
#include "onrobot_driver/IModbusConnection.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "rclcpp/rclcpp.hpp"

namespace onrobot_driver {

class GripperDetection {
public:
    static std::string detectGripperType(std::unique_ptr<IModbusConnection>& connection, int device_address = 65) {
        try {
            RCLCPP_INFO(rclcpp::get_logger("GripperDetection"), "Attempting to detect gripper type...");
            
            // Read product code from register 1536 (0x600)
            MB::ModbusRequest req(device_address, MB::utils::ReadAnalogOutputHoldingRegisters, 1536, 1);
            MB::ModbusResponse resp = connection->sendRequest(req);
            
            if (resp.registerValues().empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("GripperDetection"), "No response from gripper");
                return "";
            }
            
            uint16_t product_code = resp.registerValues().front().isReg() ? 
                                   resp.registerValues().front().reg() : 0;
            
            RCLCPP_INFO(rclcpp::get_logger("GripperDetection"), "Detected product code: 0x%04X", product_code);
            
            // Map product code to gripper type
            return productCodeToType(product_code);
            
        } catch (const MB::ModbusException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("GripperDetection"), 
                        "Modbus exception during detection: %s", e.what());
            return "";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("GripperDetection"), 
                        "Exception during gripper detection: %s", e.what());
            return "";
        }
    }

private:
    static std::string productCodeToType(uint16_t product_code) {
        switch (product_code) {
            case 0x10: return "vg10";
            case 0x11: return "vgc10";
            case 0x18: return "vgp20";
            case 0x20: return "rg2";
            case 0x21: return "rg6";
            case 0x22: return "rg2-ft";
            case 0x50: return "sg";
            case 0x70: return "3fg15";
            case 0x71: return "3fg25";
            case 0x80: return "screwdriver";
            case 0xA0: return "mg10";
            case 0xB0: return "sander";
            case 0xC0: return "2fg7";
            case 0xC1: return "2fg14";
            case 0xF0: return "2fgp20";
            case 0x100: return "lift100_v1";
            case 0x101: return "lift100_v2";
            default: 
                RCLCPP_WARN(rclcpp::get_logger("GripperDetection"), 
                           "Unknown product code: 0x%04X", product_code);
                return "";
        }
    }
};

} // namespace onrobot_driver