#include "Arduino.h"
#include "Motor.h"

Motor::Motor(const int a, const int b, const int p, const int ch) 
    : pin_a(a), pin_b(b), pwm(p), pwm_channel(ch){}

void Motor::initMotor() {
    pinMode(pin_a, OUTPUT);
    pinMode(pin_b, OUTPUT);
    ledcSetup(pwm_channel, 5000, 8);
    ledcAttachPin(pwm, pwm_channel);
}


