#include <Arduino.h>

#include <Wire.h>
#include <7Semi_BNO055.h>

#define TXPIN 17
#define RXPIN 16
#define BAUDRATE 115200
#define UARTID 2

HardwareSerial picolink(UARTID);
hw_timer_t *Timer0_Cfg = NULL;
BNO055_7Semi imu;

bool sendThrusterFrame = false;
void IRAM_ATTR Timer0_ISR()
{
    if (!armingSequence && !sendThrusterFrame)
    {
        sendThrusterFrame = true;
    }
}

uint16_t escframegen(uint16_t throttle)
{
    throttle &= 0x7FF;
    uint16_t packet = (throttle << 1) | 0;
    unsigned int crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    uint16_t escframe = (packet << 4) | crc;
    return escframe;
}

bool armingSequence = true;
void setup()
{
    Serial.begin(115200);
    delay(200);
    // PICO setup
    {
        Serial.begin(115200);
        picolink.begin(BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);
        picolink.write(0b10010000);
        delay(1000);
        armingSequence = false;
    }
    
    // Timer setup
    {
        Timer0_Cfg = timerBegin(0, 80, true);
        timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
        timerAlarmWrite(Timer0_Cfg, 1000, true);
        timerAlarmEnable(Timer0_Cfg);
    }

    // IMU setup
    {
        if (!imu.begin(Wire, 32, 33, /*useExtCrystal=*/true))
        {
            Serial.println(F("BNO055 not found"));
            while (1)
                delay(1000);
        }

        imu.setMode(Mode::NDOF);
        Serial.print(F("Calibrating"));
        if (!imu.waitCalibrated(10000, 200))
        {
            Serial.println(F(" - timeout"));
        }
        else
        {
            Serial.println(F(" - done"));
        }

        printCalib();
        Serial.println(F("Ready!\n"));
        // Optional: wait for calibration
        Serial.print(F("Calibrating"));
        if (!imu.waitCalibrated(10000, 200))
        {
            Serial.println(F(" - timeout"));
        }

        else
        {
            Serial.println(F(" - done"));
        }

        printCalib();

        Serial.println(F("Ready!\n"));
    }
    
}

void send_escframe(uint16_t escframe)
{
    uint8_t dom = (escframe >> 8) & 0xFF;
    uint8_t sub = escframe & 0xFF;
    picolink.write(dom);
    picolink.write(sub);
}

void loop()
{
    if (sendThrusterFrame)
    {
        uint16_t throttle[5] = {1347, 1347, 1347, 1347, 1347};

        for (int i = 0; i < 5; i++)
        {
            picolink.write(0b00010000 | i);
            send_escframe(escframegen(throttle[i]));
        }

        sendThrusterFrame = false;
    }
}
