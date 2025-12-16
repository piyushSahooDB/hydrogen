#include "BmsLink.hpp"

// -------- Public Functions --------
BmsLink::BmsLink(HardwareSerial& serial, uint32_t baud) {
    m_serial = &serial;
    m_baud = baud;
    m_sequence = 0;
    m_errorPending = false;
    m_rxIndex = 0;
    m_errorByte = 0;
}

void BmsLink::begin() {
    m_serial->begin(m_baud);
}

void BmsLink::update() {
    // Non-blocking RX parsing
    parseRx();

    // Non-blocking error frame sending
    if (m_errorPending && m_serial->availableForWrite() >= 11) { 
        // Only send if there is enough room in TX buffer
        sendPendingError();
    }
}

void BmsLink::sendTelemetry(const TelemetryData& data) {
    uint8_t frame[11];
    frame[0] = data.sequence;
    frame[1] = data.current_mA & 0xFF;
    frame[2] = (data.current_mA >> 8) & 0xFF;
    frame[3] = data.output_mV & 0xFF;
    frame[4] = (data.output_mV >> 8) & 0xFF;
    frame[5] = data.total_mV & 0xFF;
    frame[6] = (data.total_mV >> 8) & 0xFF;
    frame[7] = data.temp_centiC & 0xFF;
    frame[8] = (data.temp_centiC >> 8) & 0xFF;
    frame[9] = data.error;

    uint8_t crc = crc8(frame, 10);
    frame[10] = crc;

    m_serial->write(0xAE); // SOF
    m_serial->write(frame, sizeof(frame));
}

void BmsLink::pushError(uint8_t errorFlags) {
    m_errorByte = errorFlags;
    m_errorPending = true;
}

// -------- Private Functions --------
void BmsLink::parseRx() {
    while (m_serial->available()) {
        uint8_t b = m_serial->read();

        if (m_rxIndex == 0 && b != 0xAE) continue; // look for SOF

        m_rxBuffer[m_rxIndex++] = b;

        if (m_rxIndex >= 4) {
            uint8_t cmd = m_rxBuffer[1];
            uint8_t notCmd = m_rxBuffer[2];

            if ((cmd ^ notCmd) == 0xFF) { // valid inverse
                if (validateCommand(cmd)) sendAck();
                else sendNack();
            }

            m_rxIndex = 0; // reset buffer
        }
    }
}

bool BmsLink::validateCommand(uint8_t cmd) {
    // TODO: implement full command validation (STOP_ELECTRONICS, START_THRUSTERS, etc.)
    return true;
}

void BmsLink::sendAck() {
    m_serial->write(0xA5);
}

void BmsLink::sendNack() {
    m_serial->write(0x5A);
}

void BmsLink::sendPendingError() {
    if (m_errorPending) {
        TelemetryData errFrame{};
        errFrame.sequence = m_sequence++;
        errFrame.error = m_errorByte;
        sendTelemetry(errFrame);
        m_errorPending = false;
    }
}
