#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "onrobot_driver/IModbusConnection.hpp"
#include "onrobot_driver/TCPConnectionWrapper.hpp"
#include "onrobot_driver/SerialConnectionWrapper.hpp"
// Modbus request/response definitions and utilities
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class FG {
public:
    // Constructors for TCP and Serial connections
    FG(const std::string &type, const std::string &ip, int port);

    // Serial: type + device + baudrate (device_address = 65 fijo)
    //FG(const std::string &type, const std::string &device, int baudrate);
    ~FG();

    // Read commands
    float getWidth();                    // En metros (diámetro externo para 3FG15)
    std::vector<int> getStatus();
    uint16_t getStatusRaw();
   
    // Write commands
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val); // Ancho o diámetro en metros
    void setTargetSpeed(float speed_val);
    void setCommand(uint16_t command);
   
    // Gripper control commands (mantener compatibilidad)
    void gripExternal();  // Para 2FG7/2FG14: agarre externo; para 3FG15: no aplica (ignorar o mapear a interno)
    void gripInternal();  // Para 3FG15: agarre centrípeto
    void stop();
    void moveGripper(float width_val, bool external_grip = true);

    // Utility functions
    float getMinWidth();
    float getMaxWidth();

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    int device_address_;

    // --- Parámetros por tipo de gripper ---
    struct GripperParams {
        float max_width;      // metros
        float min_width;
        float max_force;      // Newtons
        float default_force;
        float default_speed;
    };

    GripperParams params;

    void initParams(); // Inicializa según tipo

    // --- Registros Modbus (actualmente para 3fg15)
    static constexpr uint16_t REG_TARGET_WIDTH = 1; //diameter for 3fg15
    static constexpr uint16_t REG_TARGET_FORCE = 0;
    static constexpr uint16_t REG_TARGET_SPEED = 2;
    static constexpr uint16_t REG_COMMAND = 3;
    static constexpr uint16_t REG_STATUS = 256;
    static constexpr uint16_t REG_EXTERNAL_WIDTH = 257;  // 3FG15: diámetro externo
    static constexpr uint16_t REG_INTERNAL_WIDTH = 258;  // No usado en 3FG15
    static constexpr uint16_t REG_FORCE = 263;

    // --- Comandos ---
    static constexpr uint16_t CMD_GRIP_EXTERNAL = 1;
    static constexpr uint16_t CMD_GRIP_INTERNAL = 2;
    static constexpr uint16_t CMD_STOP = 3;

    // --- Status bits (comunes) ---
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_ERROR_NOT_CALIBRATED = 0x0008;
    static constexpr uint16_t STATUS_ERROR_LINEAR_SENSOR = 0x0010;

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
   
    // Conversión de unidades (1/10 mm <-> metros)
    float fromTenthMM(uint16_t tenth_mm);
    uint16_t toTenthMM(float meters);


    float getCurrentDiameter();           // 257: diámetro actual (m)
    float getDiameterWithOffset();        // 258: diámetro con offset (m)
    float getAppliedForce();              // 259: fuerza aplicada (N)
    float getMinDiameter();               // 513: diámetro mínimo (m)
    float getMaxDiameter();               // 514: diámetro máximo (m)
    float getFingerLength();              // 270: longitud de dedos (mm)
    float getFingerPosition();            // 272: posición de dedos (mm)
    float getFingertipOffset();           // 273: offset de puntas (mm)

    // --- Estado detallado ---
    struct GripperStatus {
        bool busy;
        bool grip_detected;
        bool force_grip_detected;
        bool calibration_ok;
    };

    GripperStatus getDetailedStatus();

    // --- Setters para configuración avanzada (3FG15) ---
    void setGripType(bool internal);      // 2: 0=externo, 1=interno
    void setFingerLength(float mm);
    void setFingerPosition(float mm);
    void setFingertipOffset(float mm);

};