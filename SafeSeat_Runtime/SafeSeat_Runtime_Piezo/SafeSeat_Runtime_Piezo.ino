#include <Arduino.h>

#include "Config.h"
#include "PiezoSensor.h"
#include "PiezoFeatureExtractor.h"


// ============================================================
// MODULES
// ============================================================

PiezoSensor piezo;
PiezoFeatureExtractor featureExtractor;


// ============================================================
// FEATURE STATE
// ============================================================

PiezoFeatures latestFeatures;

bool featureVectorReady =
    false;

unsigned long featureWindowCount =
    0;


// ============================================================
// WORKING BUFFERS
//
// PiezoSensor owns the only rolling 750-sample acquisition
// buffer.
//
// These arrays are temporary working copies used only when a
// new 5-second feature stride is due.
// ============================================================

float respirationWindow[
    PIEZO_WINDOW_SAMPLES
];

float normalizedWindow[
    PIEZO_WINDOW_SAMPLES
];


// ============================================================
// SERIAL REPORTING
// ============================================================

unsigned long lastReportMillis =
    0;


// ============================================================
// FEATURE GENERATION
// ============================================================

void processFeatureWindow()
{
    const PiezoReading &p =
        piezo.getReading();


    if (
        !p.newFeatureWindowDue
    )
    {
        return;
    }


    if (
        !piezo.copyRespirationWindow(
            respirationWindow,
            PIEZO_WINDOW_SAMPLES
        )
    )
    {
        piezo.acknowledgeFeatureWindow();

        return;
    }


    // --------------------------------------------------------
    // Runtime robust normalization bridge
    //
    // Training used median/MAD robust standardization.
    // In training this was calculated over a complete subject
    // recording. A live system cannot access future samples,
    // so the same formula is applied to the current 30-second
    // live window.
    //
    // This is a deployment adaptation to validate later using
    // actual collected SafeSeat Piezo data.
    // --------------------------------------------------------

    bool normalized =
        featureExtractor.robustNormalizeWindow(
            respirationWindow,
            normalizedWindow,
            PIEZO_WINDOW_SAMPLES
        );


    if (
        !normalized
    )
    {
        featureVectorReady =
            false;


        piezo.acknowledgeFeatureWindow();

        return;
    }


    bool extracted =
        featureExtractor.computeFeatures(
            normalizedWindow,
            PIEZO_WINDOW_SAMPLES,
            latestFeatures
        );


    featureVectorReady =
        extracted;


    if (
        extracted
    )
    {
        featureWindowCount++;
    }


    piezo.acknowledgeFeatureWindow();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );


    delay(
        1000
    );


    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " SafeSeat Seatbelt Runtime"
    );

    Serial.println(
        " Stage 2 - Piezo Feature Extraction"
    );

    Serial.println(
        "=========================================="
    );


    bool piezoReady =
        piezo.begin();


    if (
        piezoReady
    )
    {
        Serial.println(
            "[PIEZO] Acquisition ready."
        );
    }
    else
    {
        Serial.println(
            "[PIEZO] ERROR: initialization failed."
        );
    }


    Serial.println();
    Serial.println(
        "[ML] Waiting for first 30-second window..."
    );

    Serial.println(
        "[ML] New feature vector every 5 seconds thereafter."
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 25 Hz ACQUISITION
    // ========================================================

    piezo.update();


    // ========================================================
    // 30 s WINDOW / 5 s STRIDE FEATURE EXTRACTION
    // ========================================================

    processFeatureWindow();


    // ========================================================
    // SERIAL DASHBOARD
    // ========================================================

    unsigned long now =
        millis();


    if (
        now -
        lastReportMillis
        <
        PIEZO_SERIAL_REPORT_INTERVAL_MS
    )
    {
        return;
    }


    lastReportMillis =
        now;


    const PiezoReading &p =
        piezo.getReading();


    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " SAFESEAT PIEZO ESP32"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // ACQUISITION
    // ========================================================

    Serial.println(
        "Acquisition:"
    );


    Serial.print(
        "  Status         : "
    );


    switch (
        p.status
    )
    {
        case PiezoStatus::INITIALIZING:
            Serial.println(
                "INITIALIZING"
            );
            break;

        case PiezoStatus::READY:
            Serial.println(
                "READY"
            );
            break;

        case PiezoStatus::INVALID_READING:
            Serial.println(
                "INVALID READING"
            );
            break;
    }


    Serial.print(
        "  Sample rate    : "
    );

    Serial.print(
        p.actualSamplingRateHz,
        2
    );

    Serial.println(
        " Hz"
    );


    Serial.print(
        "  Sample count   : "
    );

    Serial.println(
        p.sampleCount
    );


    // ========================================================
    // SIGNAL
    // ========================================================

    Serial.println();
    Serial.println(
        "Signal:"
    );


    Serial.print(
        "  Raw ADC        : "
    );

    Serial.println(
        p.rawADC
    );


    Serial.print(
        "  Filtered       : "
    );

    Serial.println(
        p.filteredSignal,
        2
    );


    Serial.print(
        "  Baseline       : "
    );

    Serial.println(
        p.baseline,
        2
    );


    Serial.print(
        "  Resp waveform  : "
    );

    Serial.println(
        p.respirationWave,
        2
    );


    // ========================================================
    // BREATHING CONTEXT
    // ========================================================

    Serial.println();
    Serial.println(
        "Breathing context:"
    );


    Serial.print(
        "  Total breaths  : "
    );

    Serial.println(
        p.totalBreaths
    );


    Serial.print(
        "  Peak-based RR  : "
    );


    if (
        isfinite(
            p.estimatedRespirationBPM
        )
    )
    {
        Serial.print(
            p.estimatedRespirationBPM,
            1
        );

        Serial.println(
            " BPM"
        );
    }
    else
    {
        Serial.println(
            "not enough breaths"
        );
    }


    Serial.print(
        "  No-breath time : "
    );

    Serial.print(
        p.noBreathDurationMs /
        1000.0f,
        1
    );

    Serial.println(
        " sec"
    );


    Serial.print(
        "  15s timer      : "
    );

    Serial.println(
        p.noBreathTimerExceeded
            ? "EXCEEDED"
            : "NO"
    );


    // ========================================================
    // MODEL WINDOW
    // ========================================================

    Serial.println();
    Serial.println(
        "ML window:"
    );


    Serial.print(
        "  Samples        : "
    );

    Serial.print(
        p.windowSamplesAvailable
    );

    Serial.print(
        " / "
    );

    Serial.println(
        PIEZO_WINDOW_SAMPLES
    );


    Serial.print(
        "  30s ready      : "
    );

    Serial.println(
        p.fullWindowReady
            ? "YES"
            : "NO"
    );


    Serial.print(
        "  Feature vectors: "
    );

    Serial.println(
        featureWindowCount
    );


    // ========================================================
    // 16 FEATURES
    // ========================================================

    if (
        featureVectorReady
    )
    {
        Serial.println();
        Serial.println(
            "Latest normalized 30s features:"
        );


        Serial.print(
            "  01 mean        : "
        );
        Serial.println(
            latestFeatures.mean,
            6
        );


        Serial.print(
            "  02 std         : "
        );
        Serial.println(
            latestFeatures.std,
            6
        );


        Serial.print(
            "  03 min         : "
        );
        Serial.println(
            latestFeatures.minimum,
            6
        );


        Serial.print(
            "  04 max         : "
        );
        Serial.println(
            latestFeatures.maximum,
            6
        );


        Serial.print(
            "  05 range       : "
        );
        Serial.println(
            latestFeatures.range,
            6
        );


        Serial.print(
            "  06 median      : "
        );
        Serial.println(
            latestFeatures.median,
            6
        );


        Serial.print(
            "  07 IQR         : "
        );
        Serial.println(
            latestFeatures.iqr,
            6
        );


        Serial.print(
            "  08 RMS         : "
        );
        Serial.println(
            latestFeatures.rms,
            6
        );


        Serial.print(
            "  09 energy      : "
        );
        Serial.println(
            latestFeatures.energy,
            6
        );


        Serial.print(
            "  10 meanAbsDiff : "
        );
        Serial.println(
            latestFeatures.meanAbsDiff,
            6
        );


        Serial.print(
            "  11 stdDiff     : "
        );
        Serial.println(
            latestFeatures.stdDiff,
            6
        );


        Serial.print(
            "  12 ZCR         : "
        );
        Serial.println(
            latestFeatures.zeroCrossingRate,
            6
        );


        Serial.print(
            "  13 domFreq     : "
        );
        Serial.print(
            latestFeatures.dominantFrequencyHz,
            6
        );
        Serial.println(
            " Hz"
        );


        Serial.print(
            "  14 spectral RR : "
        );
        Serial.print(
            latestFeatures.respirationBPM,
            2
        );
        Serial.println(
            " BPM"
        );


        Serial.print(
            "  15 specEntropy : "
        );
        Serial.println(
            latestFeatures.spectralEntropy,
            6
        );


        Serial.print(
            "  16 autocorr    : "
        );
        Serial.println(
            latestFeatures.autocorrelationPeak,
            6
        );
    }
    else
    {
        Serial.println();
        Serial.println(
            "Features        : waiting for first complete window"
        );
    }


    // ========================================================
    // UPCOMING
    // ========================================================

    Serial.println();
    Serial.println(
        "------------------------------------------"
    );

    Serial.println(
        "Inference       : not integrated yet"
    );

    Serial.println(
        "PiezoComm       : not integrated yet"
    );

    Serial.println(
        "=========================================="
    );
}
