#include "MLX.h"


// ============================================================
// CONSTRUCTOR
// ============================================================

MLXSensor::MLXSensor()
{
}


// ============================================================
// INITIALIZATION
// ============================================================

bool MLXSensor::begin()
{
    reading.status =
        MLXStatus::INITIALIZING;


    Serial.println();
    Serial.println(
        "[MLX90614] Initializing..."
    );


    /*
     * Wire.begin() is handled once by the Main Hub.
     *
     * C1001 uses UART.
     * MLX90614, ADS1115, and MPU6050 all share the same
     * ESP32 I2C bus.
     */
    if (!mlx.begin())
    {
        reading.connected =
            false;

        reading.status =
            MLXStatus::DISCONNECTED;


        Serial.println(
            "[MLX90614] ERROR: sensor not detected."
        );


        return false;
    }


    reading.connected =
        true;


    resetFilter();


    Serial.println(
        "[MLX90614] Connected."
    );


    return true;
}


// ============================================================
// VALIDATION
// ============================================================

bool MLXSensor::isValidAmbient(
    float value
) const
{
    return (
        isfinite(value)
        &&
        value >= MIN_AMBIENT_C
        &&
        value <= MAX_AMBIENT_C
    );
}


bool MLXSensor::isValidObject(
    float value
) const
{
    return (
        isfinite(value)
        &&
        value >= MIN_OBJECT_C
        &&
        value <= MAX_OBJECT_C
    );
}


// ============================================================
// MEDIAN
// ============================================================

float MLXSensor::medianOfThree(
    float a,
    float b,
    float c
)
{
    if (a > b)
    {
        float temp = a;
        a = b;
        b = temp;
    }


    if (b > c)
    {
        float temp = b;
        b = c;
        c = temp;
    }


    if (a > b)
    {
        float temp = a;
        a = b;
        b = temp;
    }


    return b;
}


// ============================================================
// FILTER RESET
// ============================================================

void MLXSensor::resetFilter()
{
    filterInitialized =
        false;


    bufferIndex = 0;

    bufferCount = 0;


    filteredAmbient =
        0.0f;

    filteredObject =
        0.0f;


    for (
        int i = 0;
        i < MEDIAN_WINDOW;
        i++
    )
    {
        ambientBuffer[i] =
            0.0f;

        objectBuffer[i] =
            0.0f;
    }


    reading.rawAmbientC =
        NAN;

    reading.rawObjectC =
        NAN;


    reading.filteredAmbientC =
        NAN;

    reading.filteredObjectC =
        NAN;


    reading.objectMinusAmbientC =
        NAN;


    reading.valid =
        false;
}


// ============================================================
// INITIAL FILTER VALUE
// ============================================================

void MLXSensor::initializeFilter(
    float ambient,
    float object
)
{
    for (
        int i = 0;
        i < MEDIAN_WINDOW;
        i++
    )
    {
        ambientBuffer[i] =
            ambient;

        objectBuffer[i] =
            object;
    }


    filteredAmbient =
        ambient;

    filteredObject =
        object;


    bufferIndex = 0;

    bufferCount =
        MEDIAN_WINDOW;


    filterInitialized =
        true;


    reading.filteredAmbientC =
        filteredAmbient;

    reading.filteredObjectC =
        filteredObject;


    reading.objectMinusAmbientC =
        filteredObject
        -
        filteredAmbient;


    reading.valid =
        true;


    reading.status =
        MLXStatus::
            FILTER_INITIALIZED;
}


// ============================================================
// FILTER PROCESSING
// ============================================================

void MLXSensor::processReading(
    float ambient,
    float object
)
{
    if (!filterInitialized)
    {
        initializeFilter(
            ambient,
            object
        );

        return;
    }


    ambientBuffer[
        bufferIndex
    ] = ambient;


    objectBuffer[
        bufferIndex
    ] = object;


    bufferIndex++;


    if (
        bufferIndex >=
        MEDIAN_WINDOW
    )
    {
        bufferIndex = 0;
    }


    float medianAmbient =
        medianOfThree(
            ambientBuffer[0],
            ambientBuffer[1],
            ambientBuffer[2]
        );


    float medianObject =
        medianOfThree(
            objectBuffer[0],
            objectBuffer[1],
            objectBuffer[2]
        );


    filteredAmbient =
        EMA_ALPHA
        *
        medianAmbient
        +
        (
            1.0f
            -
            EMA_ALPHA
        )
        *
        filteredAmbient;


    filteredObject =
        EMA_ALPHA
        *
        medianObject
        +
        (
            1.0f
            -
            EMA_ALPHA
        )
        *
        filteredObject;


    reading.filteredAmbientC =
        filteredAmbient;


    reading.filteredObjectC =
        filteredObject;


    /*
     * Runtime ambient compensation/context.
     *
     * We preserve this because MLX object temperature can shift
     * when cabin/environment temperature changes.
     *
     * However:
     * the WESAD model itself was trained using TEMP behavior,
     * not MLX ambient readings.
     */
    reading.objectMinusAmbientC =
        filteredObject
        -
        filteredAmbient;


    reading.valid =
        true;


    reading.status =
        MLXStatus::TRUSTED;
}


// ============================================================
// UPDATE
// ============================================================

void MLXSensor::update()
{
    if (!reading.connected)
    {
        reading.status =
            MLXStatus::DISCONNECTED;

        return;
    }


    unsigned long now =
        millis();


    if (
        now
        -
        lastSampleTime
        <
        SAMPLE_INTERVAL_MS
    )
    {
        return;
    }


    lastSampleTime =
        now;


    // --------------------------------------------------------
    // RAW SENSOR READ
    // --------------------------------------------------------

    float rawAmbient =
        mlx.readAmbientTempC();


    float rawObject =
        mlx.readObjectTempC();


    reading.rawAmbientC =
        rawAmbient;


    reading.rawObjectC =
        rawObject;


    // --------------------------------------------------------
    // REJECT INVALID SENSOR OUTPUT
    // --------------------------------------------------------

    if (
        !isValidAmbient(
            rawAmbient
        )
        ||
        !isValidObject(
            rawObject
        )
    )
    {
        reading.valid =
            false;


        reading.status =
            MLXStatus::
                INVALID_READING;


        /*
         * Do NOT contaminate the median/EMA state.
         * Keep the last trusted filtered values available.
         */

        return;
    }


    // --------------------------------------------------------
    // FILTER
    // --------------------------------------------------------

    processReading(
        rawAmbient,
        rawObject
    );
}


// ============================================================
// GETTERS
// ============================================================

const MLXReading&
MLXSensor::getReading() const
{
    return reading;
}


bool MLXSensor::hasValidReading() const
{
    return (
        reading.connected
        &&
        reading.valid
        &&
        isfinite(
            reading.filteredObjectC
        )
        &&
        isfinite(
            reading.filteredAmbientC
        )
    );
}


// ============================================================
// STATUS
// ============================================================

const char*
MLXSensor::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case MLXStatus::DISCONNECTED:
            return "DISCONNECTED";

        case MLXStatus::INITIALIZING:
            return "INITIALIZING";

        case MLXStatus::INVALID_READING:
            return "INVALID READING - HOLD";

        case MLXStatus::FILTER_INITIALIZED:
            return "FILTER INITIALIZED";

        case MLXStatus::TRUSTED:
            return "FILTERED AND TRUSTED";

        default:
            return "UNKNOWN";
    }
}