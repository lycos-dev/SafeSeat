#pragma once

#include <Arduino.h>
#include <Adafruit_MLX90614.h>


enum class MLXStatus
{
    DISCONNECTED,
    INITIALIZING,
    INVALID_READING,
    FILTER_INITIALIZED,
    TRUSTED
};


struct MLXReading
{
    bool connected = false;
    bool valid = false;

    float rawAmbientC = NAN;
    float rawObjectC = NAN;

    float filteredAmbientC = NAN;
    float filteredObjectC = NAN;

    // Runtime environmental context.
    //
    // IMPORTANT:
    // This is NOT a WESAD-trained feature by itself.
    float objectMinusAmbientC = NAN;

    MLXStatus status =
        MLXStatus::DISCONNECTED;
};


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
    // WESAD E4 TEMP model was engineered at 4 Hz.
    // 250 ms = 4 samples/sec.
    // ========================================================

    static constexpr unsigned long
        SAMPLE_INTERVAL_MS = 250UL;


    unsigned long lastSampleTime = 0;


    // ========================================================
    // FILTER
    //
    // Preserved from your tested MLX implementation:
    // 3-sample median -> EMA(alpha = 0.35)
    // ========================================================

    static constexpr int
        MEDIAN_WINDOW = 3;

    static constexpr float
        EMA_ALPHA = 0.35f;


    float ambientBuffer[
        MEDIAN_WINDOW
    ] = {0};

    float objectBuffer[
        MEDIAN_WINDOW
    ] = {0};


    int bufferIndex = 0;

    int bufferCount = 0;


    bool filterInitialized = false;


    float filteredAmbient = 0.0f;

    float filteredObject = 0.0f;


    // ========================================================
    // SENSOR-SPECIFICATION VALIDITY LIMITS
    //
    // These reject impossible/invalid MLX output.
    // They are NOT medical thresholds.
    // ========================================================

    static constexpr float
        MIN_AMBIENT_C = -40.0f;

    static constexpr float
        MAX_AMBIENT_C = 85.0f;


    static constexpr float
        MIN_OBJECT_C = -40.0f;

    static constexpr float
        MAX_OBJECT_C = 125.0f;


    bool isValidAmbient(
        float value
    ) const;


    bool isValidObject(
        float value
    ) const;


    float medianOfThree(
        float a,
        float b,
        float c
    );


    void resetFilter();


    void initializeFilter(
        float ambient,
        float object
    );


    void processReading(
        float ambient,
        float object
    );
};