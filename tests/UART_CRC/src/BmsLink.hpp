#pragma once

#include <Arduino.h>
#include "crc8.hpp"

struct TelemetryData {
    uint8_t sequence;
    int16_t current_mA;
    uint16_t output_mV;
    uint16_t total_mV;
    int16_t temp_centiC;
    uint8_t error;
};

class BmsLink {
public:
    // -------- Public Variables --------

private:
    // -------- Private Variables --------
    HardwareSerial* m_serial;
    uint32_t m_baud;
    uint8_t m_sequence;

    uint8_t m_errorByte;
    bool m_errorPending;

    uint8_t m_rxBuffer[32];
    size_t m_rxIndex;

public:
    // -------- Public Functions --------
    BmsLink(HardwareSerial& serial, uint32_t baud);

    void begin();
    void update();
    void sendTelemetry(const TelemetryData& data);
    void pushError(uint8_t errorFlags);

private:
    // -------- Private Functions --------
    void parseRx();
    bool validateCommand(uint8_t cmd);
    void sendAck();
    void sendNack();
    void sendPendingError();
};
