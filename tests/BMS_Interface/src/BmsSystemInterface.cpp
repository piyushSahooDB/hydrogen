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
