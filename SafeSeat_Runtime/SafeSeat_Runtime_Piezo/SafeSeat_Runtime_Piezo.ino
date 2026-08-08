#include <Arduino.h>

#include "Config.h"
#include "PiezoSensor.h"
#include "PiezoFeatureExtractor.h"
#include "PiezoInference.h"


// ============================================================
// MODULES
// ============================================================

PiezoSensor piezo;
PiezoFeatureExtractor featureExtractor;
PiezoInference piezoInference;


// ============================================================
// FEATURE / INFERENCE STATE
// ============================================================

PiezoFeatures latestFeatures;
PiezoInferenceResult latestInference;

bool featureVectorReady =
    false;

bool inferenceReady =
    false;

unsigned long featureWindowCount =
    0;


// ============================================================
// WORKING BUFFERS
//
// PiezoSensor owns the persistent rolling 750-sample window.
// These are temporary working arrays only when a feature
// stride is due.
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
// WINDOW -> FEATURES -> MODEL
// ============================================================

void processModelWindow()
{
    const PiezoReading &p =
        piezo.getReading();

    if (
        !p.newFeatureWindowDue
    )
    {
        return;
    }

    inferenceReady =
        false;

    featureVectorReady =
        false;

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
    // DEPLOYMENT NORMALIZATION BRIDGE
    //
    // The training pipeline robust-standardized each complete
    // WESAD subject recording using median + MAD.
    //
    // A live device cannot see an entire future recording, so
    // the same robust formula is applied to the current
    // 30-second live window.
    //
    // This adaptation must be validated with collected
    // SafeSeat Piezo recordings before final model claims.
    // --------------------------------------------------------

    if (
        !featureExtractor.robustNormalizeWindow(
            respirationWindow,
            normalizedWindow,
            PIEZO_WINDOW_SAMPLES
        )
    )
    {
        piezo.acknowledgeFeatureWindow();
        return;
    }

    if (
        !featureExtractor.computeFeatures(
            normalizedWindow,
            PIEZO_WINDOW_SAMPLES,
            latestFeatures
        )
    )
    {
        piezo.acknowledgeFeatureWindow();
        return;
    }

    featureVectorReady =
        true;

    featureWindowCount++;

    inferenceReady =
        piezoInference.predict(
            latestFeatures,
            latestInference
        );

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
        " Stage 3 - Embedded Piezo Inference"
    );

    Serial.println(
        "=========================================="
    );

    bool piezoReady =
        piezo.begin();

    Serial.println(
        piezoReady
            ? "[PIEZO] Acquisition ready."
            : "[PIEZO] ERROR: initialization failed."
    );

    Serial.println();
    Serial.println(
        "[ML] Model source: tuned SafeSeat PIEZO joblib models."
    );

    Serial.println(
        "[ML] First result after 30 seconds."
    );

    Serial.println(
        "[ML] New result every 5 seconds thereafter."
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    piezo.update();

    processModelWindow();

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
    // SENSOR
    // ========================================================

    Serial.println(
        "Acquisition:"
    );

    Serial.print(
        "  Sample rate     : "
    );

    Serial.print(
        p.actualSamplingRateHz,
        2
    );

    Serial.println(
        " Hz"
    );

    Serial.print(
        "  Samples         : "
    );

    Serial.println(
        p.sampleCount
    );

    Serial.print(
        "  Resp waveform   : "
    );

    Serial.println(
        p.respirationWave,
        2
    );

    Serial.print(
        "  30s window      : "
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

    // ========================================================
    // AUXILIARY BREATH CONTEXT
    // ========================================================

    Serial.println();
    Serial.println(
        "Breathing context:"
    );

    Serial.print(
        "  Peak-based RR   : "
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
            "not ready"
        );
    }

    Serial.print(
        "  No-breath time  : "
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
        "  15s timer       : "
    );

    Serial.println(
        p.noBreathTimerExceeded
            ? "EXCEEDED"
            : "NO"
    );

    // ========================================================
    // FEATURE VECTOR
    // ========================================================

    Serial.println();
    Serial.println(
        "Model window:"
    );

    Serial.print(
        "  Feature windows : "
    );

    Serial.println(
        featureWindowCount
    );

    if (
        featureVectorReady
    )
    {
        Serial.print(
            "  Spectral RR     : "
        );

        Serial.print(
            latestFeatures.respirationBPM,
            2
        );

        Serial.println(
            " BPM"
        );

        Serial.print(
            "  Spectral entropy: "
        );

        Serial.println(
            latestFeatures.spectralEntropy,
            6
        );

        Serial.print(
            "  Autocorr peak   : "
        );

        Serial.println(
            latestFeatures.autocorrelationPeak,
            6
        );
    }
    else
    {
        Serial.println(
            "  Features         : waiting"
        );
    }

    // ========================================================
    // TRAINED MODEL INFERENCE
    // ========================================================

    Serial.println();
    Serial.println(
        "Trained anomaly models:"
    );

    if (
        inferenceReady &&
        latestInference.valid
    )
    {
        Serial.print(
            "  IsolationForest : "
        );

        Serial.print(
            latestInference.isolationForestDecision,
            6
        );

        Serial.print(
            " -> "
        );

        Serial.println(
            latestInference.isolationForestAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.print(
            "  One-Class SVM   : "
        );

        Serial.print(
            latestInference.oneClassSVMDecision,
            6
        );

        Serial.print(
            " -> "
        );

        Serial.println(
            latestInference.oneClassSVMAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.print(
            "  BOTH models     : "
        );

        Serial.println(
            latestInference.bothModelsAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.print(
            "  EITHER model    : "
        );

        Serial.println(
            latestInference.eitherModelAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );
    }
    else
    {
        Serial.println(
            "  Inference        : waiting for first window"
        );
    }

    Serial.println();
    Serial.println(
        "PiezoComm        : next stage"
    );

    Serial.println(
        "=========================================="
    );
}
