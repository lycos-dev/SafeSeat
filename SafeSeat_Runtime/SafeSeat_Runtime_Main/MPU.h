#pragma once

#include <Arduino.h>

enum class MPUStatus
{
    DISCONNECTED,
    INITIALIZING,
    READY,
    INVALID_READING
};

struct MPUReading
{
    bool connected = false;
    bool valid = false;

    // Raw MPU6050 values
    int16_t rawAx = 0;
    int16_t rawAy = 0;
    int16_t rawAz = 0;

    int16_t rawGx = 0;
    int16_t rawGy = 0;
    int16_t rawGz = 0;

    int16_t rawTemperature = 0;

    // Converted values
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;

    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    // Derived values
    float accelMagnitude = 0.0f;
    float gyroMagnitude = 0.0f;
    float dynamicAcceleration = 0.0f;

    // Timing
    unsigned long sampleCount = 0;
    float actualSamplingRateHz = 0.0f;

    MPUStatus status = MPUStatus::DISCONNECTED;
};

class MPUSensor
{
public:
    MPUSensor();

    bool begin();
    void update();

    const MPUReading& getReading() const;

    bool hasValidReading() const;

    const char* getStatusText() const;

private:
    static constexpr uint32_t SAMPLE_INTERVAL_US = 10000UL;

    // +/-2g
    static constexpr float ACCEL_SCALE = 16384.0f;

    // +/-250 degrees/sec
    static constexpr float GYRO_SCALE = 131.0f;

    MPUReading reading;

    uint32_t lastSampleMicros = 0;

    bool writeRegister(
        uint8_t reg,
        uint8_t value
    );

    bool readMotionRegisters(
        int16_t& ax,
        int16_t& ay,
        int16_t& az,
        int16_t& temperature,
        int16_t& gx,
        int16_t& gy,
        int16_t& gz
    );

    bool testConnection();

    void calculateDerivedValues();
};