#include <Arduino.h>
#define LED 2
 
hw_timer_t *Timer0_Cfg = NULL;
 
void IRAM_ATTR Timer0_ISR()
{
    digitalWrite(LED, !digitalRead(LED));
}
void setup()
{
    pinMode(LED, OUTPUT);
    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);
    timerAlarmEnable(Timer0_Cfg);
}
void loop()
{
    // Do Nothing!
}