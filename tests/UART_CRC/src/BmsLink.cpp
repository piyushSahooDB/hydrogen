#include "BmsLink.hpp"

// -------- Public Functions --------
BmsLink::BmsLink(HardwareSerial& serial, uint32_t baud) {
    m_currentCommand = NONE;
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
    parseRx();

    if (m_errorPending && m_serial->availableForWrite() >= 11) {
        sendPendingError();
    }

    if(m_currentCommand != NONE) {
        switch(m_currentCommand) {
            case STOP_ELECTRONICS:
                if(execStopElectronics()) {m_currentCommand = NONE;} //returns true on success
                break;
            case STOP_THRUSTERS:
                if(execStopThrusters()) {m_currentCommand = NONE;}
                break;
            case START_THRUSTERS:
                if(execStartThrusters()) {m_currentCommand = NONE;}
                break;
            case TELEMETRY:
                if(execTelemetry()) {m_currentCommand = NONE;}
                break;
        }
    }
}

void BmsLink::sendTelemetry() {
    m_telemetry.sequence++;

    uint8_t frame[12];   
    frame[0] = 0xAE; 
    frame[1] = m_telemetry.sequence;
    frame[2] = m_telemetry.current_mA & 0xFF;
    frame[3] = (m_telemetry.current_mA >> 8) & 0xFF;
    frame[4] = m_telemetry.output_mV & 0xFF;
    frame[5] = (m_telemetry.output_mV >> 8) & 0xFF;
    frame[6] = m_telemetry.total_mV & 0xFF;
    frame[7] = (m_telemetry.total_mV >> 8) & 0xFF;
    frame[8] = m_telemetry.temp_centiC & 0xFF;
    frame[9] = (m_telemetry.temp_centiC >> 8) & 0xFF;
    frame[10] = m_telemetry.error;

    // CRC does not include SOF, so start from frame[1] and length = 10
    frame[11] = crc8(&frame[1], 10);

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

        // Wait for SOF
        if (m_rxIndex == 0 && b != 0xAE) continue;

        m_rxBuffer[m_rxIndex++] = b;

        // Minimal command frame size: 4 bytes
        if (m_rxIndex >= 4) {
            uint8_t cmd     = m_rxBuffer[1];
            uint8_t notCmd  = m_rxBuffer[2];
            uint8_t crc     = m_rxBuffer[3];

            // Verify command inverse
            if ((cmd ^ notCmd) == 0xFF) {
                bool crcValid = true;

                // Only check CRC for commands that require it
                if (commandRequiresCrc(cmd)) {
                    crcValid = (crc == crc8(m_rxBuffer, 3)); 
                    // CRC8 over first 3 bytes (cmd + notCmd), per your protocol
                }

                if (crcValid && validateCommand(cmd)) {
                    sendAck();
                    m_currentCommand = static_cast<Command>(cmd);
                }
                else sendNack();
            }

            m_rxIndex = 0; // reset buffer for next frame
        }
    }
}


bool BmsLink::validateCommand(uint8_t incommingCmd) {
    auto newCmd = static_cast<Command>(incommingCmd);
    if(m_currentCommand == STOP_ELECTRONICS) {
        return false;
    }
    else if(m_currentCommand == STOP_THRUSTERS) {
        if(newCmd == STOP_ELECTRONICS) {
            return true;
        } else {
            return false;
        }
    }
    else if(m_currentCommand == START_THRUSTERS) {
        if(newCmd == STOP_ELECTRONICS || newCmd == STOP_THRUSTERS) {
            return true;
        } else {
            return false;
        }
    }

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
        sendTelemetry();
        m_errorPending = false;
    }
}

bool BmsLink::execStopElectronics() {
    return true;
}
bool BmsLink::execStopThrusters() {
    return true;
}
bool BmsLink::execStartThrusters() {
    return true;
}
bool BmsLink::execTelemetry() {
    return true;
}
