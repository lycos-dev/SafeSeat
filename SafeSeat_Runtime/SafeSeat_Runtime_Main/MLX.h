#pragma once

#include <Arduino.h>
#include <Adafruit_MLX90614.h>


// ============================================================
// MLX90614 STATUS
// ============================================================

enum class MLXStatus
{
    DISCONNECTED,

    INITIALIZING,

    // Current physical read was invalid, but a previously
    // trusted filtered value may still be retained.
    INVALID_READING,

    FILTER_INITIALIZED,

    TRUSTED,

    HOLDING_LAST_VALUE
};


// ============================================================
// MLX90614 READING
// ============================================================

struct MLXReading
{
    bool connected = false;

    // True once a trusted filtered temperature pair exists.
    //
    // IMPORTANT:
    // An isolated invalid raw read does not erase the previous
    // trusted filtered values.
    bool valid = false;


    // --------------------------------------------------------
    // Most recent raw hardware read
    // --------------------------------------------------------

    float rawAmbientC = NAN;

    float rawObjectC = NAN;


    // --------------------------------------------------------
    // Trusted filtered values
    //
    // Proven filter:
    // 3-sample median -> EMA(alpha = 0.35)
    // --------------------------------------------------------

    float filteredAmbientC = NAN;

    float filteredObjectC = NAN;


    // --------------------------------------------------------
    // Runtime ambient context
    //
    // This is intentionally retained for SafeSeat deployment.
    // It is environmental context, not a claim that WESAD
    // directly trained on MLX ambient temperature.
    // --------------------------------------------------------

    float objectMinusAmbientC = NAN;


    // --------------------------------------------------------
    // Current-sample validity
    //
    // This tells Fusion later whether the NEW physical MLX
    // sample was accepted or whether the module is holding the
    // previous trusted value.
    // --------------------------------------------------------

    bool currentSampleAccepted = false;


    unsigned long acceptedSampleCount = 0;

    unsigned long rejectedSampleCount = 0;


    MLXStatus status =
        MLXStatus::DISCONNECTED;
};


// ============================================================
// SENSOR MODULE
// ============================================================

class MLXSensor
{
public:

    MLXSensor();


    bool begin();


    void update();


    const MLXReading&
    getReading() const;


    bool hasValidReading() const;


    const char*
    getStatusText() const;


private:

    Adafruit_MLX90614 mlx;


    MLXReading reading;


    // ========================================================
    // SAMPLING
    //
    // 4 Hz runtime temperature acquisition.
    // ========================================================

    static constexpr unsigned long
        SAMPLE_INTERVAL_MS =
            250UL;


    unsigned long lastSampleTime =
        0;


    // ========================================================
    // PROVEN FILTER SETTINGS
    //
    // Copied from the old combined sketch:
    //
    // MLX_MEDIAN_WINDOW = 3
    // MLX_EMA_ALPHA     = 0.35
    // ========================================================

    static constexpr int
        MEDIAN_WINDOW =
            3;


    static constexpr float
        EMA_ALPHA =
            0.35f;


    float ambientBuffer[
        MEDIAN_WINDOW
    ] = {0};


    float objectBuffer[
        MEDIAN_WINDOW
    ] = {0};


    int bufferIndex =
        0;


    bool filterInitialized =
        false;


    float filteredAmbient =
        0.0f;


    float filteredObject =
        0.0f;


    // ========================================================
    // SENSOR-SPECIFICATION VALIDITY LIMITS
    //
    // Same limits as the proven combined sketch.
    // They reject impossible sensor output; they are NOT
    // medical thresholds.
    // ========================================================

    static constexpr float
        MIN_AMBIENT_C =
            -40.0f;


    static constexpr float
        MAX_AMBIENT_C =
            85.0f;


    static constexpr float
        MIN_OBJECT_C =
            -40.0f;


    static constexpr float
        MAX_OBJECT_C =
            125.0f;


    bool isValidAmbient(
        float value
    ) const;


    bool isValidObject(
        float value
    ) const;


    static float medianOfThree(
        float firstValue,
        float secondValue,
        float thirdValue
    );


    void resetFilter();


    bool updateFilter(
        float rawAmbient,
        float rawObject
    );
};
