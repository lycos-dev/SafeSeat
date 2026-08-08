#pragma once

#include <Arduino.h>
#include <DFRobot_HumanDetection.h>

struct C1001Reading {
    bool connected = false;
    bool present = false;
    bool warmedUp = false;
    bool validVitals = false;

    int motionStatus = 0;
    int moveRange = 0;

    float respirationRaw = NAN;
    float heartRateRaw = NAN;

    float respirationFiltered = NAN;
    float heartRateFiltered = NAN;
};

class C1001Sensor {
public:
    C1001Sensor();

    bool begin();

    void update();

    const C1001Reading& getReading() const;

private:
    DFRobot_HumanDetection human;
    C1001Reading reading;

    bool timerStarted = false;
    unsigned long validVitalsStart = 0;

    static constexpr int MEDIAN_SIZE = 5;

    float rrBuffer[MEDIAN_SIZE] = {0};
    float hrBuffer[MEDIAN_SIZE] = {0};

    int bufferCount = 0;
    int bufferIndex = 0;

    bool emaInitialized = false;

    float rrEMA = 0.0f;
    float hrEMA = 0.0f;

    static constexpr float EMA_ALPHA = 0.35f;

    bool isValidVital(float value) const;

    void pushVitalSample(
        float rr,
        float hr
    );

    float medianOf(
        const float* values,
        int count
    );

    void resetWarmup();
};