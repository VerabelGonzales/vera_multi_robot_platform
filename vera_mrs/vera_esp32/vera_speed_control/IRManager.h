#ifndef IRMANAGER_H
#define IRMANAGER_H

#include <Arduino.h>

#define MAX_NUM_SENSORS 4
#define NUM_READINGS 10

class IRManager {
private:
    const int numSensors;
    const int* sensorPins;
    int readings[MAX_NUM_SENSORS][NUM_READINGS];
    int readIndex[MAX_NUM_SENSORS];
    int total[MAX_NUM_SENSORS];
    int average[MAX_NUM_SENSORS];
    float kalmanValue[MAX_NUM_SENSORS];
    const float processVar;
    const float sensorVar;
    float estimateErr[MAX_NUM_SENSORS];
    float kalmanGain[MAX_NUM_SENSORS];
    float distances[MAX_NUM_SENSORS];

    float convertVoltageToDistance(float voltage);

public:
    IRManager(int nSensors, const int* pins, float pVar, float sVar);

    void update();
    float* getDistances();
};

#endif // IRMANAGER_H
