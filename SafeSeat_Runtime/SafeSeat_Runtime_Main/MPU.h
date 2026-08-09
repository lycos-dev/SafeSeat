#pragma once

#include <Arduino.h>


// ============================================================
// MPU6050 STATUS
// ============================================================

enum class MPUStatus
{
    DISCONNECTED,
    INITIALIZING,
    READY,
    INVALID_READING
};


// ============================================================
// MPU6050 READING
// ============================================================

struct MPUReading
{
    bool connected = false;
    bool valid = false;

    // Raw register values
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

    float temperatureC = 0.0f;

    // Derived runtime context
    float accelMagnitude = 0.0f;
    float gyroMagnitude = 0.0f;
    float dynamicAcceleration = 0.0f;

    // Timing
    unsigned long sampleCount = 0;
    float actualSamplingRateHz = 0.0f;

    MPUStatus status =
        MPUStatus::DISCONNECTED;
};


// ============================================================
// SENSOR CLASS
// ============================================================

class MPUSensor
{
public:
    MPUSensor();

    bool begin();

    void update();

    // Reset rate diagnostics after other blocking startup
    // calibration work has completed.
    void resetSamplingDiagnostics();

    const MPUReading&
    getReading() const;

    bool hasValidReading() const;

    const char*
    getStatusText() const;


private:
    // --------------------------------------------------------
    // Proven combined-sketch scales:
    //
    // MPU6050 accel config 0x00 -> +/-2g -> 16384 LSB/g
    // MPU6050 gyro config  0x00 -> +/-250 dps -> 131 LSB/dps
    // --------------------------------------------------------

    static constexpr float
        ACCEL_SCALE =
            16384.0f;

    static constexpr float
        GYRO_SCALE =
            131.0f;


    // Acquisition request interval remains 10 ms.
    //
    // In the full SafeSeat main-loop workload this request has
    // empirically yielded ~80-81 Hz actual MPU acquisition, which
    // is the cadence used for Step 5.6 model retraining.
    //
    // We intentionally preserve the proven 10 ms request here
    // instead of blindly changing it to 12.5 ms; doing so would
    // risk pushing the real shared-loop cadence below 80 Hz.
    // The final all-sensor test will verify Actual Fs again.
    static constexpr uint32_t
        SAMPLE_INTERVAL_US =
            10000UL;


    MPUReading reading;


    uint32_t lastSampleMicros =
        0;


    uint32_t acquisitionStartMicros =
        0;


    // ========================================================
    // PROVEN RAW I2C ROUTINES
    // ========================================================

    void writeRegister(
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


    void calculateConvertedValues();

    void calculateDerivedValues();

    void updateSamplingRate(
        uint32_t nowMicros
    );
};
