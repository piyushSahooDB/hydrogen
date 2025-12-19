#include "BmsSystemInterface.hpp"

BmsSystemInterface::BmsSystemInterface(HardwareSerial& serial, uint32_t baud, Pins pins) {
    m_serial = &serial;
    m_baud = baud;
    m_pins = pins;
    m_lastCommand = NONE;
}

void BmsSystemInterface::begin() {
    m_serial->begin(
        m_baud,
        SERIAL_8N1,
        m_pins.BMS.rx,
        m_pins.BMS.tx
    );
}

void BmsSystemInterface::update() {

}

bool BmsSystemInterface::sendCommand(Command cmd) {
    uint8_t frame[4];
    frame[0] = 0xAE;           
    frame[1] = static_cast<uint8_t>(cmd);  
    frame[2] = ~frame[1];      
    frame[3] = crc8(frame + 1, 2); 

    size_t written = m_serial->write(frame, sizeof(frame));
    m_lastCommand = cmd;

    if (cmd == STOP_ELECTRONICS) return true;

    return written == sizeof(frame);
}

bool BmsSystemInterface::stopElectronics() {
    return sendCommand(STOP_ELECTRONICS);
}

bool BmsSystemInterface::stopThrusters() {
    return sendCommand(STOP_THRUSTERS);
}

bool BmsSystemInterface::startThrusters() {
    return sendCommand(START_THRUSTERS);
}

bool BmsSystemInterface::requestTelemetry() {
    return sendCommand(TELEMETRY);
}

void BmsSystemInterface::parseRx() {
    while (m_serial->available()) {
        uint8_t b = m_serial->read();

        // --- ACK ---
        if (b == 0xA5) {
            m_ackReceived  = true;
            m_nackReceived = false;
            continue;
        }

        // --- NACK ---
        if (b == 0x5A) {
            m_ackReceived  = false;
            m_nackReceived = true;
            continue;
        }

        // --- Telemetry framing ---
        if (m_rxIndex == 0) {
            if (b != 0xAE) {
                continue; // hunt SOF
            }
        }

        m_rxBuffer[m_rxIndex++] = b;

        // Telemetry frame = 12 bytes total
        if (m_rxIndex == 12) {
            handleTelemetryFrame();
            m_rxIndex = 0;
        }

        // Overflow protection
        if (m_rxIndex >= sizeof(m_rxBuffer)) {
            m_rxIndex = 0;
        }
    }
}

void BmsSystemInterface::handleTelemetryFrame() {
    uint8_t* payload = &m_rxBuffer[1];

    uint8_t receivedCrc = m_rxBuffer[11];
    uint8_t computedCrc = crc8(payload, 10);

    if (receivedCrc != computedCrc) {
        // CRC fail → drop frame silently
        return;
    }

    TelemetryData& t = latestTelemetry;

    t.sequence = payload[0];

    t.current_centiA =
        (int16_t)(payload[1] | (payload[2] << 8));

    t.output_mV =
        (uint16_t)(payload[3] | (payload[4] << 8));

    t.total_mV =
        (uint16_t)(payload[5] | (payload[6] << 8));

    t.temp_centiC =
        (int16_t)(payload[7] | (payload[8] << 8));

    t.error = payload[9];
}
