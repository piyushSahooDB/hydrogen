#pragma once

#include <Arduino.h>
#include "crc8.hpp"
#include "Pins.hpp"

struct TelemetryData {
    uint8_t sequence;
    int16_t current_centiA;
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

class BmsSystemInterface {
public:
    // -------- Public Variables --------
    TelemetryData latestTelemetry;
    bool lastCommandAcked;

private:
    HardwareSerial* m_serial;
    uint32_t m_baud;
    Pins m_pins;

    uint8_t m_rxBuffer[32];
    size_t m_rxIndex;
    bool m_ackReceived;
    bool m_nackReceived;

    Command m_lastCommand;

public:
    BmsSystemInterface(HardwareSerial& serial, uint32_t baud, Pins pins);

    void begin();
    void update(); 

    bool sendCommand(Command cmd);
    bool stopElectronics();
    bool stopThrusters();
    bool startThrusters();
    bool requestTelemetry();

private:
    void parseRx();
    void handleTelemetryFrame();
    void handleAck();
    void handleNack();
};
