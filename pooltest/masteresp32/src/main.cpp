#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <7Semi_BNO055.h>

#define TXPIN 17
#define RXPIN 16
#define BAUDRATE 115200
#define UARTID 2

HardwareSerial picolink(UARTID);
hw_timer_t* Timer0_Cfg = NULL;
BNO055_7Semi imu;

bool sendThrusterFrame = false;
bool sendImuTelemetry = false;

uint8_t imu_counter = 0;
void IRAM_ATTR Timer0_ISR()
{
    if (!armingSequence && !sendThrusterFrame)
    {
        sendThrusterFrame = true;
    }

    if (imuCalibrated && !sendImuTelemety)
    {
        updateThrusters();

        if (imu_counter == 10) {
            sendImuTelemetry = true;
        }
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
bool imuCalibrated = false;
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
    }

    // Enable interrupts
    {
        armingSequence = false;
        imuCalibrated = true;
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
        uint16_t throttle[5] = { 1347, 1347, 1347, 1347, 1347 };

        for (int i = 0; i < 5; i++)
        {
            picolink.write(0b00010000 | i);
            send_escframe(escframegen(throttle[i]));
        }

        sendThrusterFrame = false;
    }

    if (sendImuTelemetry)
    {
        float heading, roll, pitch;
        if (imu.readEuler(heading, roll, pitch))
        {
            Serial.print(F("Euler H/R/P: "));
            Serial.print(heading, 1);
            Serial.print('\t');
            Serial.print(roll, 1);
            Serial.print('\t');
            Serial.println(pitch, 1);
        }

        // Raw sensor data
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        if (imu.readAccel(ax, ay, az))
        {
            Serial.print(F("Accel raw: "));
            Serial.print(ax);
            Serial.print(", ");
            Serial.print(ay);
            Serial.print(", ");
            Serial.println(az);
        }
        if (imu.readGyro(gx, gy, gz))
        {
            Serial.print(F("Gyro raw : "));
            Serial.print(gx);
            Serial.print(", ");
            Serial.print(gy);
            Serial.print(", ");
            Serial.println(gz);
        }
        if (imu.readMag(mx, my, mz))
        {
            Serial.print(F("Mag  raw : "));
            Serial.print(mx);
            Serial.print(", ");
            Serial.print(my);
            Serial.print(", ");
            Serial.println(mz);
        }

        // Optional: linear acceleration / gravity
        int16_t lx, ly, lz, gxv, gyv, gzv;
        if (imu.readLinear(lx, ly, lz))
        {
            Serial.print(F("LinAcc raw: "));
            Serial.print(lx);
            Serial.print(", ");
            Serial.print(ly);
            Serial.print(", ");
            Serial.println(lz);
        }
        if (imu.readGravity(gxv, gyv, gzv))
        {
            Serial.print(F("Grav  raw: "));
            Serial.print(gxv);
            Serial.print(", ");
            Serial.print(gyv);
            Serial.print(", ");
            Serial.println(gzv);
        }

        // Optional: quaternion
        float qw, qx, qy, qz;
        if (imu.readQuat(qw, qx, qy, qz))
        {
            Serial.print(F("Quat WXYZ: "));
            Serial.print(qw, 4);
            Serial.print(", ");
            Serial.print(qx, 4);
            Serial.print(", ");
            Serial.print(qy, 4);
            Serial.print(", ");
            Serial.println(qz, 4);
        }
        int temp = imu.temperatureC();
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println(" °C");
        // Show calibration every ~2s
        static uint32_t tCal = 0;
        if (millis() - tCal > 2000)
        {
            tCal = millis();
            printCalib();
        }

        Serial.println();

        sendImuTelemetry = false;
    }
}
