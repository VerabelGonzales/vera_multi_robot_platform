#include "IRManager.h"

IRManager::IRManager(int nSensors, const int* pins, float pVar, float sVar) 
    : numSensors(nSensors), sensorPins(pins), processVar(pVar), sensorVar(sVar) {
    
    for (int i = 0; i < numSensors; i++) {
        estimateErr[i] = 1;
        readIndex[i] = 0;
        total[i] = 0;
        for (int j = 0; j < NUM_READINGS; j++) {
            readings[i][j] = 0;
        }
    }
}

void IRManager::update() {
    for (int i = 0; i < numSensors; i++) {
        // Actualizar el promedio móvil
        total[i] -= readings[i][readIndex[i]];
        readings[i][readIndex[i]] = analogRead(sensorPins[i]);
        total[i] += readings[i][readIndex[i]];
        average[i] = total[i] / NUM_READINGS;
        readIndex[i]++;
        if (readIndex[i] >= NUM_READINGS) {
            readIndex[i] = 0;
        }

        float voltage = average[i] * (3.3 / 4095.0);

        // Aplicar el filtro de Kalman
        kalmanGain[i] = estimateErr[i] / (estimateErr[i] + sensorVar);
        kalmanValue[i] += kalmanGain[i] * (voltage - kalmanValue[i]);
        estimateErr[i] = (1 - kalmanGain[i]) * estimateErr[i] + abs(kalmanValue[i] - voltage) * processVar;

        distances[i] = convertVoltageToDistance(kalmanValue[i]);
    }
}

float* IRManager::getDistances() {
    return distances;
}

float IRManager::convertVoltageToDistance(float voltage) {
    return (27.728 * pow(voltage, -1.2045));
}
