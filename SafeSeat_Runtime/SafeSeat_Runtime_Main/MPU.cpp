#include "MPU.h"
#include "Config.h"

#include <Wire.h>
#include <math.h>


// ============================================================
// CONSTRUCTOR
// ============================================================

MPUSensor::MPUSensor()
{
}


// ============================================================
// RAW REGISTER WRITE
//
// Direct modular equivalent of:
//
// void mpu6050Write(uint8_t reg, uint8_t data)
//
// from the proven combined sketch.
// ============================================================

void MPUSensor::writeRegister(
    uint8_t reg,
    uint8_t value
)
{
    Wire.beginTransmission(
        MPU6050_ADDRESS
    );

    Wire.write(
        reg
    );

    Wire.write(
        value
    );

    Wire.endTransmission();
}


// ============================================================
// RAW 14-BYTE MOTION READ
//
// Exact pattern from the proven combined sketch:
//
// start at 0x3B
// repeated start
// request 14 bytes
// Ax Ay Az Temp Gx Gy Gz
// ============================================================

bool MPUSensor::readMotionRegisters(
    int16_t& ax,
    int16_t& ay,
    int16_t& az,
    int16_t& temperature,
    int16_t& gx,
    int16_t& gy,
    int16_t& gz
)
{
    Wire.beginTransmission(
        MPU6050_ADDRESS
    );

    Wire.write(
        0x3B
    );


    if (
        Wire.endTransmission(
            false
        )
        !=
        0
    )
    {
        return false;
    }


    uint8_t received =
        Wire.requestFrom(
            MPU6050_ADDRESS,
            static_cast<uint8_t>(
                14
            ),
            static_cast<bool>(
                true
            )
        );


    if (
        received
        !=
        14
    )
    {
        return false;
    }


    ax =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    ay =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    az =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    temperature =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    gx =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    gy =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    gz =
        static_cast<int16_t>(
            (
                Wire.read()
                <<
                8
            )
            |
            Wire.read()
        );


    return true;
}


// ============================================================
// INITIALIZATION
//
// Restores the exact successful setup sequence:
//
// 0x6B = 0x00 -> wake
// 0x1B = 0x00 -> gyro +/-250 dps
// 0x1C = 0x00 -> accel +/-2g
//
// No WHO_AM_I gate and no separate connection-test stage.
// The proven combined sketch simply configured the device and
// successfully read it afterward.
// ============================================================

bool MPUSensor::begin()
{
    reading =
        MPUReading{};


    reading.status =
        MPUStatus::INITIALIZING;


    Serial.println();
    Serial.println(
        "[MPU6050] Initializing from proven combined sketch..."
    );


    // Wake MPU6050.
    writeRegister(
        0x6B,
        0x00
    );


    delay(
        10
    );


    // Gyroscope +/-250 deg/s.
    writeRegister(
        0x1B,
        0x00
    );


    // Accelerometer +/-2g.
    writeRegister(
        0x1C,
        0x00
    );


    delay(
        10
    );


    /*
     * Instead of a separate WHO_AM_I test, verify the exact
     * operation we need: reading the 14-byte motion block.
     *
     * This preserves the proven low-level behavior while still
     * allowing Runtime_Main to report a real initialization
     * failure if communication is unavailable.
     */
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temperature;
    int16_t gx;
    int16_t gy;
    int16_t gz;


    if (
        !readMotionRegisters(
            ax,
            ay,
            az,
            temperature,
            gx,
            gy,
            gz
        )
    )
    {
        reading.connected =
            false;


        reading.valid =
            false;


        reading.status =
            MPUStatus::DISCONNECTED;


        Serial.println(
            "[MPU6050] ERROR: 14-byte motion read failed."
        );


        return false;
    }


    reading.connected =
        true;


    reading.valid =
        true;


    reading.rawAx =
        ax;

    reading.rawAy =
        ay;

    reading.rawAz =
        az;

    reading.rawTemperature =
        temperature;

    reading.rawGx =
        gx;

    reading.rawGy =
        gy;

    reading.rawGz =
        gz;


    calculateConvertedValues();

    calculateDerivedValues();


    acquisitionStartMicros =
        micros();


    lastSampleMicros =
        acquisitionStartMicros;


    reading.status =
        MPUStatus::READY;


    Serial.println(
        "MPU6050 (0x68) Ready"
    );


    return true;
}


// ============================================================
// CONVERSION
//
// Exact scales from the proven combined sketch.
// ============================================================

void MPUSensor::calculateConvertedValues()
{
    reading.accelX =
        static_cast<float>(
            reading.rawAx
        )
        /
        ACCEL_SCALE;


    reading.accelY =
        static_cast<float>(
            reading.rawAy
        )
        /
        ACCEL_SCALE;


    reading.accelZ =
        static_cast<float>(
            reading.rawAz
        )
        /
        ACCEL_SCALE;


    reading.gyroX =
        static_cast<float>(
            reading.rawGx
        )
        /
        GYRO_SCALE;


    reading.gyroY =
        static_cast<float>(
            reading.rawGy
        )
        /
        GYRO_SCALE;


    reading.gyroZ =
        static_cast<float>(
            reading.rawGz
        )
        /
        GYRO_SCALE;


    reading.temperatureC =
        (
            static_cast<float>(
                reading.rawTemperature
            )
            /
            340.0f
        )
        +
        36.53f;
}


// ============================================================
// DERIVED VALUES
//
// These are retained from the modular runtime because they are
// useful later for vibration gating / fusion. They sit ON TOP
// of the restored proven raw acquisition.
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


    /*
     * Simple gravity-relative dynamic acceleration context:
     * absolute deviation of total acceleration from 1 g.
     *
     * This is NOT the trained MPU feature vector itself.
     * It is runtime context useful for later sensor fusion.
     */
    reading.dynamicAcceleration =
        fabsf(
            reading.accelMagnitude
            -
            1.0f
        );
}


// ============================================================
// SAMPLE RATE
// ============================================================

void MPUSensor::updateSamplingRate(
    uint32_t nowMicros
)
{
    uint32_t elapsed =
        nowMicros
        -
        acquisitionStartMicros;


    if (
        elapsed
        ==
        0
    )
    {
        reading.actualSamplingRateHz =
            0.0f;


        return;
    }


    reading.actualSamplingRateHz =
        (
            static_cast<float>(
                reading.sampleCount
            )
            *
            1000000.0f
        )
        /
        static_cast<float>(
            elapsed
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
        reading.valid =
            false;


        reading.status =
            MPUStatus::DISCONNECTED;


        return;
    }


    uint32_t now =
        micros();


    if (
        static_cast<uint32_t>(
            now
            -
            lastSampleMicros
        )
        <
        SAMPLE_INTERVAL_US
    )
    {
        return;
    }


    /*
     * Advance by one target interval rather than setting
     * directly to now, reducing long-term cadence drift.
     */
    lastSampleMicros +=
        SAMPLE_INTERVAL_US;


    /*
     * If something blocked the MCU for a long period, avoid
     * attempting a rapid catch-up burst.
     */
    if (
        static_cast<uint32_t>(
            now
            -
            lastSampleMicros
        )
        >
        SAMPLE_INTERVAL_US
        *
        4UL
    )
    {
        lastSampleMicros =
            now;
    }


    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temperature;
    int16_t gx;
    int16_t gy;
    int16_t gz;


    if (
        !readMotionRegisters(
            ax,
            ay,
            az,
            temperature,
            gx,
            gy,
            gz
        )
    )
    {
        reading.valid =
            false;


        reading.status =
            MPUStatus::INVALID_READING;


        return;
    }


    reading.rawAx =
        ax;

    reading.rawAy =
        ay;

    reading.rawAz =
        az;

    reading.rawTemperature =
        temperature;

    reading.rawGx =
        gx;

    reading.rawGy =
        gy;

    reading.rawGz =
        gz;


    calculateConvertedValues();

    calculateDerivedValues();


    reading.sampleCount++;


    updateSamplingRate(
        now
    );


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
// STATUS TEXT
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
