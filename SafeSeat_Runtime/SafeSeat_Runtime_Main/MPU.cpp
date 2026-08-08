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
// LOW-LEVEL REGISTER WRITE
// ============================================================

bool MPUSensor::writeRegister(
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

    return (
        Wire.endTransmission()
        ==
        0
    );
}


// ============================================================
// CONNECTION TEST
// ============================================================

bool MPUSensor::testConnection()
{
    // WHO_AM_I register = 0x75

    Wire.beginTransmission(
        MPU6050_ADDRESS
    );

    Wire.write(
        0x75
    );

    if (
        Wire.endTransmission(
            false
        )
        != 0
    )
    {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            MPU6050_ADDRESS,
            (uint8_t)1,
            (bool)true
        );

    if (
        received != 1
    )
    {
        return false;
    }

    uint8_t whoAmI =
        Wire.read();

    /*
     * MPU6050 typically returns 0x68.
     */
    return (
        whoAmI == 0x68
    );
}


// ============================================================
// READ ALL MOTION REGISTERS
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
    /*
     * ACCEL_XOUT_H starts at 0x3B.
     *
     * 14 consecutive bytes:
     *
     * AX H/L
     * AY H/L
     * AZ H/L
     * TEMP H/L
     * GX H/L
     * GY H/L
     * GZ H/L
     */

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
        != 0
    )
    {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            MPU6050_ADDRESS,
            (uint8_t)14,
            (bool)true
        );

    if (
        received != 14
    )
    {
        return false;
    }

    ax =
        (Wire.read() << 8)
        |
        Wire.read();

    ay =
        (Wire.read() << 8)
        |
        Wire.read();

    az =
        (Wire.read() << 8)
        |
        Wire.read();

    temperature =
        (Wire.read() << 8)
        |
        Wire.read();

    gx =
        (Wire.read() << 8)
        |
        Wire.read();

    gy =
        (Wire.read() << 8)
        |
        Wire.read();

    gz =
        (Wire.read() << 8)
        |
        Wire.read();

    return true;
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
        "[MPU6050] Initializing using raw I2C..."
    );

    if (
        !testConnection()
    )
    {
        reading.connected =
            false;

        reading.status =
            MPUStatus::DISCONNECTED;

        Serial.println(
            "[MPU6050] ERROR: device not detected at 0x68."
        );

        return false;
    }


    // ========================================================
    // WAKE SENSOR
    //
    // PWR_MGMT_1 = 0x6B
    // Writing 0 removes sleep mode.
    // ========================================================

    if (
        !writeRegister(
            0x6B,
            0x00
        )
    )
    {
        reading.connected =
            false;

        reading.status =
            MPUStatus::DISCONNECTED;

        Serial.println(
            "[MPU6050] ERROR: failed to wake sensor."
        );

        return false;
    }


    delay(
        100
    );


    // ========================================================
    // ACCELEROMETER RANGE
    //
    // ACCEL_CONFIG = 0x1C
    // 0x00 = +/-2g
    // ========================================================

    writeRegister(
        0x1C,
        0x00
    );


    // ========================================================
    // GYROSCOPE RANGE
    //
    // GYRO_CONFIG = 0x1B
    // 0x00 = +/-250 deg/s
    // ========================================================

    writeRegister(
        0x1B,
        0x00
    );


    // ========================================================
    // DIGITAL LOW PASS FILTER
    //
    // CONFIG = 0x1A
    //
    // DLPF_CFG = 3
    // Approx:
    // accel BW ~44 Hz
    // gyro BW ~42 Hz
    //
    // Good compromise for road vibration acquisition.
    // ========================================================

    writeRegister(
        0x1A,
        0x03
    );


    // ========================================================
    // SAMPLE RATE
    //
    // SMPLRT_DIV = 0x19
    //
    // With DLPF enabled:
    // internal rate = 1 kHz
    //
    // 1000 / (1 + 9) = 100 Hz
    // ========================================================

    writeRegister(
        0x19,
        9
    );


    reading.connected =
        true;

    reading.valid =
        false;

    reading.sampleCount =
        0;

    lastSampleMicros =
        micros();

    reading.status =
        MPUStatus::READY;


    Serial.println(
        "[MPU6050] Connected at 0x68."
    );

    Serial.println(
        "[MPU6050] Accel range: +/-2g."
    );

    Serial.println(
        "[MPU6050] Gyro range: +/-250 deg/s."
    );

    Serial.println(
        "[MPU6050] Configured for ~100 Hz."
    );


    return true;
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


    /*
     * Stationary MPU magnitude is approximately 1g.
     *
     * Remove the static gravity component.
     */
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
        lastSampleMicros;


    if (
        elapsed <
        SAMPLE_INTERVAL_US
    )
    {
        return;
    }


    /*
     * Use actual elapsed time instead of assuming exactly 10ms.
     */
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


    lastSampleMicros =
        now;


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


    reading.rawTemperature =
        temperature;


    // ========================================================
    // UNIT CONVERSION
    //
    // These match the configured sensor ranges.
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