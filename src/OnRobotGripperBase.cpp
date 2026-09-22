#include "onrobot_driver/common/OnRobotGripperBase.hpp"

namespace onrobot_driver
{

OnRobotGripperBase::OnRobotGripperBase(int device_address)
    : device_address_(device_address),
      connection_type_("unknown"),
      target_endpoint_("unknown")
{
}

OnRobotGripperBase::~OnRobotGripperBase()
{
    close();
}

void OnRobotGripperBase::close()
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (connection_)
    {
        try
        {
            connection_->close();
        }
        catch (...)
        {
        }
        connection_.reset();
    }
}

bool OnRobotGripperBase::isConnected() const
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    return connection_ != nullptr;
}

void OnRobotGripperBase::connectTCP(const std::string &ip, int port, std::function<bool()> keep_running)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    connection_type_ = "tcp";
    target_endpoint_ = ip + ":" + std::to_string(port);

    while (true)
    {
        if (keep_running && !keep_running())
        {
            throw std::runtime_error("Connection aborted by shutdown signal");
        }

        try
        {
            connection_ = std::unique_ptr<IModbusConnection>(new TCPConnectionWrapper(ip, port));
            std::cout << "Connected to OnRobot gripper at " << target_endpoint_ << std::endl;
            break;
        }
        catch (const MB::ModbusException &ex)
        {
            std::cerr << "Waiting for OnRobot TCP connection at "
                      << target_endpoint_ << " (" << ex.what() << "). Retrying every 500ms..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

void OnRobotGripperBase::connectSerial(const std::string &device, std::function<bool()> keep_running)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    connection_type_ = "serial";
    target_endpoint_ = device;

    while (true)
    {
        if (keep_running && !keep_running())
        {
            throw std::runtime_error("Connection aborted by shutdown signal");
        }

        try
        {
            connection_ = std::unique_ptr<IModbusConnection>(new SerialConnectionWrapper(device));
            std::cout << "Connected to OnRobot gripper on " << target_endpoint_ << std::endl;
            break;
        }
        catch (const MB::ModbusException &ex)
        {
            std::cerr << "Waiting for OnRobot Serial connection on "
                      << target_endpoint_ << " (" << ex.what() << "). Retrying every 500ms..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

MB::ModbusResponse OnRobotGripperBase::sendRequest(const MB::ModbusRequest &req)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (!connection_)
    {
        throw std::runtime_error("Modbus connection is not open");
    }
    return connection_->sendRequest(req);
}

void OnRobotGripperBase::resetToolPower(int compute_box_address)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (!connection_)
    {
        throw std::runtime_error("Cannot reset tool power: no active Modbus connection");
    }

    std::vector<MB::ModbusCell> values = {MB::ModbusCell(static_cast<uint16_t>(2))};
    MB::ModbusRequest req(compute_box_address, MB::utils::WriteSingleAnalogOutputRegister, 0, 1, values);
    try
    {
        connection_->sendRequest(req);
        std::cout << "Sent Tool Power Reset command to Compute Box (address " << compute_box_address << ")" << std::endl;
    }
    catch (const MB::ModbusException &ex)
    {
        std::cerr << "Failed to reset tool power on Compute Box: " << ex.what() << std::endl;
        throw;
    }
}

std::vector<int> OnRobotGripperBase::getStatus()
{
    uint16_t raw = getStatusRaw();
    std::vector<int> bits(16, 0);
    for (int i = 0; i < 16; ++i)
    {
        bits[i] = (raw >> i) & 1;
    }
    return bits;
}

} // namespace onrobot_driver
