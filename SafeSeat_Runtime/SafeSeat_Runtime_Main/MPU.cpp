#include "MPU.h"
#include "Config.h"

#include <math.h>


// ============================================================
// CONSTRUCTOR
// ============================================================

MPUSensor::MPUSensor()
    : mpu(MPU6050_ADDRESS)
{
}


// ============================================================
// INITIALIZATION
// ============================================================

bool MPUSensor::begin()
{
    reading.status =
        MPUStatus::INITIALIZING;


    Serial.println();
    Serial.println(
        "[MPU6050] Initializing..."
    );


    /*
     * Wire has already been started by the Main Hub.
     */
    mpu.initialize();


    if (
        !mpu.testConnection()
    )
    {
        reading.connected =
            false;


        reading.status =
            MPUStatus::DISCONNECTED;


        Serial.println(
            "[MPU6050] ERROR: connection failed."
        );


        return false;
    }


    // ========================================================
    // MODEL-CONSISTENT SENSOR RANGES
    // ========================================================

    mpu.setFullScaleAccelRange(
        MPU6050_ACCEL_FS_2
    );


    mpu.setFullScaleGyroRange(
        MPU6050_GYRO_FS_250
    );


    /*
     * Digital low-pass filtering.
     *
     * This still retains road impacts while reducing
     * very high-frequency electrical/mechanical noise.
     */
    mpu.setDLPFMode(
        MPU6050_DLPF_BW_42
    );


    /*
     * MPU internal output rate:
     *
     * with DLPF enabled:
     * 1 kHz / (1 + divider)
     *
     * divider = 9
     * -> 100 Hz
     */
    mpu.setRate(
        9
    );


    reading.connected =
        true;


    reading.valid =
        false;


    reading.sampleCount =
        0;


    reading.lastSampleMicros =
        micros();


    reading.status =
        MPUStatus::READY;


    Serial.println(
        "[MPU6050] Connected."
    );


    Serial.println(
        "[MPU6050] Range: +/-2g, +/-250 deg/s."
    );


    Serial.println(
        "[MPU6050] Target sampling: 100 Hz."
    );


    return true;
}


// ============================================================
// VALIDATION
// ============================================================

bool MPUSensor::valuesAreValid(
    int16_t ax,
    int16_t ay,
    int16_t az,
    int16_t gx,
    int16_t gy,
    int16_t gz
) const
{
    /*
     * int16_t itself always lies within legal MPU output range.
     *
     * This check mainly rejects the highly suspicious situation
     * where every channel simultaneously returns zero.
     */

    return !(
        ax == 0
        &&
        ay == 0
        &&
        az == 0
        &&
        gx == 0
        &&
        gy == 0
        &&
        gz == 0
    );
}


// ============================================================
// DERIVED VALUES
// ============================================================

void MPUSensor::calculateDerivedValues()
{
    reading.accelMagnitude =
        sqrtf(
            reading.accelX
            *
            reading.accelX
            +
            reading.accelY
            *
            reading.accelY
            +
            reading.accelZ
            *
            reading.accelZ
        );


    reading.gyroMagnitude =
        sqrtf(
            reading.gyroX
            *
            reading.gyroX
            +
            reading.gyroY
            *
            reading.gyroY
            +
            reading.gyroZ
            *
            reading.gyroZ
        );


    reading.dynamicAcceleration =
        fabsf(
            reading.accelMagnitude
            -
            1.0f
        );
}


// ============================================================
// UPDATE
// ============================================================

void MPUSensor::update()
{
    if (
        !reading.connected
    )
    {
        reading.status =
            MPUStatus::DISCONNECTED;


        return;
    }


    uint32_t now =
        micros();


    uint32_t elapsed =
        now
        -
        reading.lastSampleMicros;


    if (
        elapsed <
        SAMPLE_INTERVAL_US
    )
    {
        return;
    }


    // ========================================================
    // SAMPLE RATE MEASUREMENT
    // ========================================================

    if (
        elapsed > 0
    )
    {
        reading.actualSamplingRateHz =
            1000000.0f
            /
            static_cast<float>(
                elapsed
            );
    }


    reading.lastSampleMicros =
        now;


    // ========================================================
    // MPU READ
    // ========================================================

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;


    mpu.getMotion6(
        &ax,
        &ay,
        &az,
        &gx,
        &gy,
        &gz
    );


    // ========================================================
    // VALIDITY
    // ========================================================

    if (
        !valuesAreValid(
            ax,
            ay,
            az,
            gx,
            gy,
            gz
        )
    )
    {
        reading.valid =
            false;


        reading.status =
            MPUStatus::
                INVALID_READING;


        return;
    }


    // ========================================================
    // RAW VALUES
    // ========================================================

    reading.rawAx =
        ax;

    reading.rawAy =
        ay;

    reading.rawAz =
        az;


    reading.rawGx =
        gx;

    reading.rawGy =
        gy;

    reading.rawGz =
        gz;


    // ========================================================
    // UNIT CONVERSION
    // ========================================================

    reading.accelX =
        static_cast<float>(
            ax
        )
        /
        ACCEL_SCALE;


    reading.accelY =
        static_cast<float>(
            ay
        )
        /
        ACCEL_SCALE;


    reading.accelZ =
        static_cast<float>(
            az
        )
        /
        ACCEL_SCALE;


    reading.gyroX =
        static_cast<float>(
            gx
        )
        /
        GYRO_SCALE;


    reading.gyroY =
        static_cast<float>(
            gy
        )
        /
        GYRO_SCALE;


    reading.gyroZ =
        static_cast<float>(
            gz
        )
        /
        GYRO_SCALE;


    // ========================================================
    // DERIVED MOTION VALUES
    // ========================================================

    calculateDerivedValues();


    reading.sampleCount++;


    reading.valid =
        true;


    reading.status =
        MPUStatus::READY;
}


// ============================================================
// GETTERS
// ============================================================

const MPUReading&
MPUSensor::getReading() const
{
    return reading;
}


bool MPUSensor::hasValidReading() const
{
    return (
        reading.connected
        &&
        reading.valid
    );
}


// ============================================================
// STATUS
// ============================================================

const char*
MPUSensor::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case MPUStatus::DISCONNECTED:
            return "DISCONNECTED";

        case MPUStatus::INITIALIZING:
            return "INITIALIZING";

        case MPUStatus::READY:
            return "READY";

        case MPUStatus::INVALID_READING:
            return "INVALID READING";

        default:
            return "UNKNOWN";
    }
}