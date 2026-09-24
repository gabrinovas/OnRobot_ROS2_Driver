#include <gtest/gtest.h>
#include <map>
#include <vector>
#include <memory>

#include "onrobot_driver/common/IModbusConnection.hpp"
#include "onrobot_driver/common/OnRobotGripperBase.hpp"
#include "onrobot_driver/twofg/TwoFG.hpp"
#include "onrobot_driver/threefg/ThreeFG.hpp"
#include "onrobot_driver/vgc10/VGC10.hpp"

// =============================================================================
// Mock Modbus Connection
// =============================================================================
class MockModbusConnection : public IModbusConnection {
public:
    MockModbusConnection() : closed_(false) {}
    ~MockModbusConnection() override = default;

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        recorded_requests_.push_back(req);

        uint8_t fc_raw = req.functionCode();
        auto fc = static_cast<MB::utils::MBFunctionCode>(fc_raw);
        uint16_t addr = req.registerAddress();
        uint16_t count = req.numberOfRegisters();

        if (fc == MB::utils::ReadAnalogInputRegisters || fc == MB::utils::ReadAnalogOutputHoldingRegisters) {
            std::vector<MB::ModbusCell> cells;
            for (uint16_t i = 0; i < count; ++i) {
                uint16_t val = registers_[addr + i];
                cells.emplace_back(val);
            }
            return MB::ModbusResponse(req.slaveID(), fc, addr, count, cells);
        } else if (fc == MB::utils::WriteSingleAnalogOutputRegister) {
            if (!req.registerValues().empty()) {
                registers_[addr] = req.registerValues()[0].reg();
            }
            return MB::ModbusResponse(req.slaveID(), fc, addr, 1, req.registerValues());
        } else if (fc == MB::utils::WriteMultipleAnalogOutputHoldingRegisters) {
            const auto &values = req.registerValues();
            for (size_t i = 0; i < values.size(); ++i) {
                registers_[addr + i] = values[i].reg();
            }
            return MB::ModbusResponse(req.slaveID(), fc, addr, count);
        }

        // Default response para otros códigos
        return MB::ModbusResponse(req.slaveID(), fc);
    }

    void close() override {
        closed_ = true;
    }

    void setRegister(uint16_t address, uint16_t value) {
        registers_[address] = value;
    }

    uint16_t getRegister(uint16_t address) const {
        auto it = registers_.find(address);
        return (it != registers_.end()) ? it->second : 0;
    }

    const std::vector<MB::ModbusRequest> &recordedRequests() const {
        return recorded_requests_;
    }

    void clearHistory() {
        recorded_requests_.clear();
    }

    bool isClosed() const { return closed_; }

private:
    std::map<uint16_t, uint16_t> registers_;
    std::vector<MB::ModbusRequest> recorded_requests_;
    bool closed_;
};

// =============================================================================
// Suite 1: Verificación de Mapa de Registros 2FG7
// =============================================================================
class TwoFGRegistersTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto mock = std::make_unique<MockModbusConnection>();
        mock_ptr = mock.get();
        gripper = std::make_unique<TwoFG>("2fg7", 65, std::move(mock));
    }

    MockModbusConnection *mock_ptr;
    std::unique_ptr<TwoFG> gripper;
};

TEST_F(TwoFGRegistersTest, PhysicalLimitsAndConversions) {
    EXPECT_FLOAT_EQ(gripper->getMinWidth(), 0.0f);
    EXPECT_FLOAT_EQ(gripper->getMaxWidth(), 0.07f);
    EXPECT_FLOAT_EQ(gripper->getMaxForce(), 70.0f);

    EXPECT_FLOAT_EQ(onrobot_driver::OnRobotGripperBase::fromTenthMM(350), 0.035f);
    EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.035f), 350);
}

TEST_F(TwoFGRegistersTest, ReadWidthAndForceFromRegisters) {
    // Simular registro 257 (REG_EXTERNAL_WIDTH) = 450 (45.0 mm)
    // Simular registro 263 (REG_FORCE) = 35 (35 N)
    mock_ptr->setRegister(257, 450);
    mock_ptr->setRegister(263, 35);

    EXPECT_NEAR(gripper->getWidth(), 0.045f, 1e-4f);
    EXPECT_FLOAT_EQ(gripper->getForce(), 35.0f);
}

TEST_F(TwoFGRegistersTest, StatusFlagsParsing) {
    // Simular registro 256 con BUSY (bit 0) y GRIP_DETECTED (bit 1) -> 0x0003
    mock_ptr->setRegister(256, 0x0003);

    EXPECT_EQ(gripper->getStatusRaw(), 0x0003);
    std::vector<int> status = gripper->getStatus();
    ASSERT_GE(status.size(), 2u);
    EXPECT_EQ(status[0], 1); // BUSY
    EXPECT_EQ(status[1], 1); // GRIP DETECTED
}

TEST_F(TwoFGRegistersTest, WriteTargetWidthAndForce) {
    gripper->setTargetWidth(0.040f); // 40.0 mm -> 400 tenth mm
    EXPECT_EQ(mock_ptr->getRegister(0), 400);

    gripper->setTargetForce(60.0f);
    EXPECT_EQ(mock_ptr->getRegister(1), 60);

    gripper->setTargetSpeed(80.0f);
    EXPECT_EQ(mock_ptr->getRegister(2), 80);
}

TEST_F(TwoFGRegistersTest, ControlCommands) {
    gripper->gripExternal();
    EXPECT_EQ(mock_ptr->getRegister(3), 1); // CMD_GRIP_EXTERNAL = 1

    gripper->gripInternal();
    EXPECT_EQ(mock_ptr->getRegister(3), 2); // CMD_GRIP_INTERNAL = 2

    gripper->stop();
    EXPECT_EQ(mock_ptr->getRegister(3), 3); // CMD_STOP = 3
}

// =============================================================================
// Suite 2: Verificación de Mapa de Registros 3FG15
// =============================================================================
class ThreeFGRegistersTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto mock = std::make_unique<MockModbusConnection>();
        mock_ptr = mock.get();
        gripper = std::make_unique<ThreeFG>(65, std::move(mock));
    }

    MockModbusConnection *mock_ptr;
    std::unique_ptr<ThreeFG> gripper;
};

TEST_F(ThreeFGRegistersTest, PhysicalLimitsAndCommands) {
    EXPECT_FLOAT_EQ(gripper->getMinWidth(), 0.0f);
    EXPECT_FLOAT_EQ(gripper->getMaxWidth(), 0.150f);
    EXPECT_FLOAT_EQ(gripper->getMaxForce(), 140.0f);

    // Simular registros de límites físicos en el efector (REG_MIN_DIAMETER=513, REG_MAX_DIAMETER=514)
    mock_ptr->setRegister(513, 0);     // 0.0 mm
    mock_ptr->setRegister(514, 1500);  // 150.0 mm (1500 tenth mm)

    gripper->setTargetWidth(0.090f); // 90.0 mm -> 900 tenth mm
    EXPECT_EQ(mock_ptr->getRegister(1), 900);

    // Fuerza en 3FG15: en unidades de 1/10 % (0-1000) respecto a MAX_FORCE (140 N)
    // 120 N / 140 N = 85.71% -> 857
    gripper->setTargetForce(120.0f);
    EXPECT_EQ(mock_ptr->getRegister(0), 857);

    gripper->stop();
    EXPECT_EQ(mock_ptr->getRegister(3), 0x0004); // CMD_STOP = 0x0004
}

// =============================================================================
// Suite 3: Verificación de Mapa de Registros VGC10
// =============================================================================
class VGC10RegistersTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto mock = std::make_unique<MockModbusConnection>();
        mock_ptr = mock.get();
        gripper = std::make_unique<onrobot_driver::VGC10>("vgc10", 65, std::move(mock));
    }

    MockModbusConnection *mock_ptr;
    std::unique_ptr<onrobot_driver::VGC10> gripper;
};

TEST_F(VGC10RegistersTest, ChannelAAndBVacuumControl) {
    // Grip canal A al 80% de vacío
    // En VGC10: High byte = Modo (1=Grip), Low byte = % Vacío (máx 80% según manual)
    gripper->gripChannelA(80);
    uint16_t reg0 = mock_ptr->getRegister(0);
    EXPECT_EQ((reg0 >> 8) & 0x00FF, onrobot_driver::VGC10::MODE_GRIP);
    EXPECT_EQ(reg0 & 0x00FF, 80);

    // Release canal A
    gripper->releaseChannelA();
    reg0 = mock_ptr->getRegister(0);
    EXPECT_EQ((reg0 >> 8) & 0x00FF, onrobot_driver::VGC10::MODE_RELEASE);

    // Grip ambos canales (clamped a 80% máx según especificación)
    gripper->gripAll(100);
    uint16_t regA = mock_ptr->getRegister(0);
    uint16_t regB = mock_ptr->getRegister(1);
    EXPECT_EQ((regA >> 8) & 0x00FF, onrobot_driver::VGC10::MODE_GRIP);
    EXPECT_EQ(regA & 0x00FF, 80);
    EXPECT_EQ((regB >> 8) & 0x00FF, onrobot_driver::VGC10::MODE_GRIP);
    EXPECT_EQ(regB & 0x00FF, 80);
}

TEST_F(VGC10RegistersTest, VacuumTelemetryReading) {
    // Simular registro 258 (Canal A) con 750 / 1000 (75% de vacío)
    // Simular registro 259 (Canal B) con 250 / 1000 (25% de vacío)
    mock_ptr->setRegister(258, 750);
    mock_ptr->setRegister(259, 250);

    EXPECT_NEAR(gripper->getVacuumChannelA(), 0.75f, 1e-3f);
    EXPECT_NEAR(gripper->getVacuumChannelB(), 0.25f, 1e-3f);
    EXPECT_NEAR(gripper->getWidth(), 0.50f, 1e-3f); // Media de ambos canales
}
