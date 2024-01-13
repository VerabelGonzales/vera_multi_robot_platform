#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder {
  public:
    //Constructor - pin_a, pin_b -> Canales del encoder
    Encoder(const int pin_a, const int pin_b);

    void initEncoder();

    //Motor Outputs - plus is one direction and minus is the other
    const int pin_a;
    const int pin_b;
};

#endif