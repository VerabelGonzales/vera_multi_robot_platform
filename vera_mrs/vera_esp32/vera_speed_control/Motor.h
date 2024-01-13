
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    //Constructor - pin_a, pin_b -> dirección del motor
    Motor(const int pin_a, const int pin_b, const int pwm, const int pwm_channel);
    
    // Inicializa el motor
    void initMotor();

    //Motor Outputs - plus is one direction and minus is the other
    const int pin_a;
    const int pin_b;

    //PWM Configuration
    const int pwm;
    const int pwm_channel;
};

#endif