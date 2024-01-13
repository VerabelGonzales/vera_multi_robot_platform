#include "Arduino.h"
#include "Encoder.h"

Encoder::Encoder(const int a, const int b) 
    : pin_a(a), pin_b(b) {}
void Encoder::initEncoder() {
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);
}

