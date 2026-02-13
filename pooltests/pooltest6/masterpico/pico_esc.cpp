#include "config.hpp"
#include "pico_esc.hpp"

void pico_esc::init() {
    Serial1.begin(PICO_BAUDRATE, SERIAL_8N1, PICO_RXPIN, PICO_TXPIN);
}

void pico_esc::arm_thrusters() {
    Serial1.write(0b10010000);          //address code so that pico sets esc into arming mode
}

void pico_esc::send_escframe(uint16_t throttle) {
    throttle &= 0x7FF;
    uint16_t packet = (throttle << 1) | 0;
    unsigned int crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;         //calulating 4bit CRC
    uint16_t escframe = (packet << 4) | crc;        //final 16bit frame that needs to be sent ( same as dshot protocol )

    uint8_t dom = (escframe >> 8) & 0xFF;       //repacakaging into two 8 bit packets for UART
    uint8_t sub = escframe & 0xFF;
    Serial1.write(dom);
    Serial1.write(sub);
}