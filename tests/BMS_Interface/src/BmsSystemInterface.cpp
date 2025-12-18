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

bool BmsSystemInterface::stopElectronics() {

}

bool BmsSystemInterface::stopThrusters() {

}

bool BmsSystemInterface::startThrusters() {

}

bool BmsSystemInterface::requestTelemetry() {

}
