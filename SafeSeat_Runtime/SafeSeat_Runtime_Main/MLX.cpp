#include "MLX.h"

#include <math.h>


// ============================================================
// CONSTRUCTOR
// ============================================================

MLXSensor::MLXSensor()
{
}


// ============================================================
// INITIALIZATION
//
// IMPORTANT:
// Wire.begin() belongs to SafeSeat_Runtime_Main.ino.
//
// This module uses the already-running shared I2C bus, matching
// the proven combined sketch.
// ============================================================

bool MLXSensor::begin()
{
    reading =
        MLXReading{};


    reading.status =
        MLXStatus::INITIALIZING;


    Serial.println();
    Serial.println(
        "=================================="
    );

    Serial.println(
        " MLX90614 Temperature Sensor"
    );

    Serial.println(
        "=================================="
    );


    if (
        !mlx.begin()
    )
    {
        reading.connected =
            false;


        reading.valid =
            false;


        reading.status =
            MLXStatus::DISCONNECTED;


        Serial.println(
            "[MLX90614] ERROR: sensor not detected!"
        );

        Serial.println(
            "[MLX90614] SDA -> GPIO21"
        );

        Serial.println(
            "[MLX90614] SCL -> GPIO22"
        );


        return false;
    }


    reading.connected =
        true;


    resetFilter();


    // Do not manufacture a valid value in begin().
    // The first real pair is acquired by update(), exactly as
    // the proven combined loop did.
    lastSampleTime =
        0;


    Serial.println(
        "MLX90614 Connected Successfully!"
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
        isfinite(
            value
        )
        &&
        value
        >=
        MIN_AMBIENT_C
        &&
        value
        <=
        MAX_AMBIENT_C
    );
}


bool MLXSensor::isValidObject(
    float value
) const
{
    return (
        isfinite(
            value
        )
        &&
        value
        >=
        MIN_OBJECT_C
        &&
        value
        <=
        MAX_OBJECT_C
    );
}


// ============================================================
// MEDIAN OF THREE
//
// This is copied from the proven combined sketch.
// ============================================================

float MLXSensor::medianOfThree(
    float firstValue,
    float secondValue,
    float thirdValue
)
{
    if (
        firstValue
        >
        secondValue
    )
    {
        float temporary =
            firstValue;

        firstValue =
            secondValue;

        secondValue =
            temporary;
    }


    if (
        secondValue
        >
        thirdValue
    )
    {
        float temporary =
            secondValue;

        secondValue =
            thirdValue;

        thirdValue =
            temporary;
    }


    if (
        firstValue
        >
        secondValue
    )
    {
        float temporary =
            firstValue;

        firstValue =
            secondValue;

        secondValue =
            temporary;
    }


    return secondValue;
}


// ============================================================
// FILTER RESET
//
// Same behavior as resetMLXFilter() in the proven combined
// sketch.
// ============================================================

void MLXSensor::resetFilter()
{
    filterInitialized =
        false;


    bufferIndex =
        0;


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


    reading.currentSampleAccepted =
        false;


    reading.acceptedSampleCount =
        0;


    reading.rejectedSampleCount =
        0;


    reading.valid =
        false;
}


// ============================================================
// FILTER UPDATE
//
// This is the modular equivalent of updateMLXFilter() from the
// proven combined sketch.
//
// Current invalid samples are rejected BEFORE touching the
// median/EMA state.
// ============================================================

bool MLXSensor::updateFilter(
    float rawAmbient,
    float rawObject
)
{
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
        return false;
    }


    // --------------------------------------------------------
    // First accepted pair initializes the complete 3-sample
    // median window and EMA state.
    // --------------------------------------------------------

    if (
        !filterInitialized
    )
    {
        for (
            int i = 0;
            i < MEDIAN_WINDOW;
            i++
        )
        {
            ambientBuffer[i] =
                rawAmbient;


            objectBuffer[i] =
                rawObject;
        }


        filteredAmbient =
            rawAmbient;


        filteredObject =
            rawObject;


        bufferIndex =
            0;


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
            MLXStatus::FILTER_INITIALIZED;


        return true;
    }


    // --------------------------------------------------------
    // 3-SAMPLE MEDIAN
    // --------------------------------------------------------

    ambientBuffer[
        bufferIndex
    ] =
        rawAmbient;


    objectBuffer[
        bufferIndex
    ] =
        rawObject;


    bufferIndex++;


    if (
        bufferIndex
        >=
        MEDIAN_WINDOW
    )
    {
        bufferIndex =
            0;
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


    // --------------------------------------------------------
    // EMA(alpha = 0.35)
    // --------------------------------------------------------

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


    reading.objectMinusAmbientC =
        filteredObject
        -
        filteredAmbient;


    reading.valid =
        true;


    reading.status =
        MLXStatus::TRUSTED;


    return true;
}


// ============================================================
// UPDATE
// ============================================================

void MLXSensor::update()
{
    if (
        !reading.connected
    )
    {
        reading.currentSampleAccepted =
            false;


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


    // ========================================================
    // EXACT HARDWARE READ ORDER FROM THE PROVEN COMBINED LOOP
    // ========================================================

    float rawAmbient =
        mlx.readAmbientTempC();


    float rawObject =
        mlx.readObjectTempC();


    reading.rawAmbientC =
        rawAmbient;


    reading.rawObjectC =
        rawObject;


    bool accepted =
        updateFilter(
            rawAmbient,
            rawObject
        );


    reading.currentSampleAccepted =
        accepted;


    if (
        accepted
    )
    {
        reading.acceptedSampleCount++;


        // updateFilter() already sets FILTER_INITIALIZED or
        // TRUSTED as appropriate.
        return;
    }


    // ========================================================
    // INVALID CURRENT SAMPLE
    //
    // Proven combined code returned false without modifying
    // filteredAmbientTemp/filteredObjectTemp.
    //
    // Do the same here:
    // - do not contaminate median/EMA state
    // - retain previous trusted filtered values
    // - explicitly expose that the CURRENT sample was rejected
    // ========================================================

    reading.rejectedSampleCount++;


    if (
        filterInitialized
        &&
        isfinite(
            reading.filteredAmbientC
        )
        &&
        isfinite(
            reading.filteredObjectC
        )
    )
    {
        // We still possess a valid previous filtered pair.
        reading.valid =
            true;


        reading.status =
            MLXStatus::HOLDING_LAST_VALUE;
    }
    else
    {
        // No trusted pair has ever been obtained.
        reading.valid =
            false;


        reading.status =
            MLXStatus::INVALID_READING;
    }
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
            reading.filteredAmbientC
        )
        &&
        isfinite(
            reading.filteredObjectC
        )
        &&
        isfinite(
            reading.objectMinusAmbientC
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
            return "INVALID READING - NO TRUSTED VALUE YET";


        case MLXStatus::FILTER_INITIALIZED:
            return "FILTER INITIALIZED";


        case MLXStatus::TRUSTED:
            return "FILTERED AND VALID";


        case MLXStatus::HOLDING_LAST_VALUE:
            return "INVALID CURRENT SAMPLE - HOLDING PREVIOUS VALUE";


        default:
            return "UNKNOWN";
    }
}
