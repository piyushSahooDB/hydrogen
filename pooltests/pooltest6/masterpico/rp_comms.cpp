#include "config.hpp"
#include "rp_comms.hpp"

void rp_comms::init() {
    Serial1.begin(RP_BAUDRATE, SERIAL_8N1, RP_RXPIN, RP_TXPIN);
}