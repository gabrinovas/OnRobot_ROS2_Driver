#pragma once

#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <atomic>

#include "IModbusConnection.hpp"
#include "TCPConnectionWrapper.hpp"
#include "SerialConnectionWrapper.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

namespace onrobot_driver
{

class OnRobotGripperBase
{
public:
    explicit OnRobotGripperBase(int device_address);
    virtual ~OnRobotGripperBase();

    // Connection methods with patient retry
    void connectTCP(const std::string &ip, int port, std::function<bool()> keep_running = nullptr);
    void connectSerial(const std::string &device, std::function<bool()> keep_running = nullptr);
    bool reconnect(int timeout_ms = 1000);
    void close();
    bool isConnected() const;

    // Diagnostic metrics
    uint64_t getTotalRequests() const { return total_requests_.load(); }
    uint64_t getFailedRequests() const { return failed_requests_.load(); }
    uint64_t getReconnectCount() const { return reconnect_count_.load(); }
    double getLastRoundtripMs() const { return last_roundtrip_ms_.load(); }
    const std::string &getTargetEndpoint() const { return target_endpoint_; }

    // Common Modbus request execution (thread-safe with automatic recovery)
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);

    // Compute Box / Eye Box Power Reset (Register 0x0000, Device 63)
    void resetToolPower(int compute_box_address = 63);

    // Pure virtual read commands
    virtual float getWidth() = 0;
    virtual float getForce() = 0;
    virtual uint16_t getStatusRaw() = 0;
    virtual std::vector<int> getStatus();

    // Pure virtual write commands
    virtual void setTargetWidth(float width_val) = 0;
    virtual void setTargetForce(float force_val) = 0;
    virtual void setTargetSpeed(float speed_val) = 0;
    virtual void stop() = 0;
    virtual void moveGripper(float width_val) = 0;

    // Physical limits query
    virtual float getMinWidth() const = 0;
    virtual float getMaxWidth() const = 0;
    virtual float getMaxForce() const = 0;

    // Conversion utilities (1/10 mm to meters and vice-versa)
    static float fromTenthMM(uint16_t tenth_mm)
    {
        return static_cast<float>(tenth_mm) / 10000.0f;
    }

    static uint16_t toTenthMM(float meters)
    {
        return static_cast<uint16_t>(meters * 10000.0f);
    }

    int getDeviceAddress() const { return device_address_; }
    const std::string &getConnectionType() const { return connection_type_; }

protected:
    std::unique_ptr<IModbusConnection> connection_;
    int device_address_;
    std::string connection_type_;
    std::string target_endpoint_;
    std::string ip_;
    int port_{502};
    std::string device_;

    std::atomic<uint64_t> total_requests_{0};
    std::atomic<uint64_t> failed_requests_{0};
    std::atomic<uint64_t> reconnect_count_{0};
    std::atomic<double> last_roundtrip_ms_{0.0};

    mutable std::mutex comm_mutex_;
};

} // namespace onrobot_driver
