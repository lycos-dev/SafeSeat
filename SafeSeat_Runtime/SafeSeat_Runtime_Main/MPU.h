#pragma once

#include <Arduino.h>
#include <MPU6050.h>


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

    // ========================================================
    // RAW MPU6050 VALUES
    // ========================================================

    int16_t rawAx = 0;
    int16_t rawAy = 0;
    int16_t rawAz = 0;

    int16_t rawGx = 0;
    int16_t rawGy = 0;
    int16_t rawGz = 0;


    // ========================================================
    // PHYSICAL UNITS
    //
    // Acceleration:
    //     g
    //
    // Gyroscope:
    //     degrees / second
    // ========================================================

    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;

    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;


    // ========================================================
    // DERIVED MOTION VALUES
    // ========================================================

    float accelMagnitude = 0.0f;

    float gyroMagnitude = 0.0f;

    /*
     * Gravity-compensated acceleration magnitude:
     *
     * |A_mag - 1g|
     *
     * This is useful as road-motion context.
     */
    float dynamicAcceleration = 0.0f;


    // ========================================================
    // TIMING
    // ========================================================

    unsigned long sampleCount = 0;

    unsigned long lastSampleMicros = 0;

    float actualSamplingRateHz = 0.0f;


    MPUStatus status =
        MPUStatus::DISCONNECTED;
};


class MPUSensor
{
public:

    MPUSensor();


    bool begin();


    /*
     * Call as frequently as possible.
     *
     * Internal timing limits actual hardware reads
     * to approximately 100 Hz.
     */
    void update();


    const MPUReading&
    getReading() const;


    bool hasValidReading() const;


    const char*
    getStatusText() const;


private:

    MPU6050 mpu;


    MPUReading reading;


    // ========================================================
    // MODEL-ALIGNED SAMPLING RATE
    //
    // SafeSeat MPU6050 feature engineering:
    // 100 Hz
    // 1-second windows
    // 50% overlap
    // ========================================================

    static constexpr uint32_t
        SAMPLE_INTERVAL_US = 10000UL;


    // ========================================================
    // MPU6050 SCALE
    //
    // Default:
    // Accelerometer +/-2g  -> 16384 LSB/g
    // Gyroscope +/-250 dps -> 131 LSB/(deg/s)
    // ========================================================

    static constexpr float
        ACCEL_SCALE = 16384.0f;


    static constexpr float
        GYRO_SCALE = 131.0f;


    bool valuesAreValid(
        int16_t ax,
        int16_t ay,
        int16_t az,
        int16_t gx,
        int16_t gy,
        int16_t gz
    ) const;


    void calculateDerivedValues();
};