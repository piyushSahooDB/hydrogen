#pragma once

#include <Arduino.h>
#include "crc8.hpp"

struct TelemetryData {
    uint8_t sequence;
    int16_t current_centiA; //centi amps
    uint16_t output_mV;
    uint16_t total_mV;
    int16_t temp_centiC;
    uint8_t error;
};

enum Command : uint8_t {
    NONE             = 0x00,
    STOP_ELECTRONICS = 0xC0,
    STOP_THRUSTERS   = 0xC7,  
    START_THRUSTERS  = 0xB8,
    TELEMETRY        = 0xBF
};

class BmsLink {
public:
    // -------- Public Variables --------

private:
    // -------- Private Variables --------
    HardwareSerial* m_serial;
    uint32_t m_baud;
    uint8_t m_sequence;
    Command m_currentCommand;

    uint8_t m_errorByte;
    bool m_errorPending;

    uint8_t m_rxBuffer[32];
    size_t m_rxIndex;

    uint8_t m_seq;
    TelemetryData m_telemetry;

public:
    // -------- Public Functions --------
    BmsLink(HardwareSerial& serial, uint32_t baud);

    void begin();
    void update();
    void sendTelemetry();
    void pushError(uint8_t errorFlags);

private:
    // -------- Private Functions --------
    void parseRx();
    bool validateCommand(uint8_t cmd);
    void sendAck();
    void sendNack();
    void sendPendingError();
    void readSensors();

    bool execStopElectronics();
    bool execStopThrusters();
    bool execStartThrusters();
    bool execTelemetry();
};
