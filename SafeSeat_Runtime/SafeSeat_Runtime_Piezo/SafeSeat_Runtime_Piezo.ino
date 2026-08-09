#include <Arduino.h>

#include "Config.h"
#include "PiezoSensor.h"
#include "PiezoSignalProcessor.h"
#include "PiezoFeatureExtractor.h"
#include "PiezoInference.h"
#include "PiezoComm.h"


// ============================================================
// SAFESEAT PIEZO / SEATBELT ESP32
//
// STEP 5.7.2
//
// PVDF @ 25 Hz
// -> exact Step 5.7 30-second window preprocessing
// -> 16 features
// -> Isolation Forest + One-Class SVM
// -> validated evidence packet
// -> Main Hub Fusion
//
// WESAD RespiBAN remains a surrogate respiratory-motion
// training source. The model output is therefore supporting
// respiration-pattern evidence, not a medical diagnosis.
// ============================================================


PiezoSensor piezo;
PiezoSignalProcessor signalProcessor;
PiezoFeatureExtractor featureExtractor;
PiezoInference piezoInference;
PiezoComm piezoComm;


PiezoFeatures latestFeatures;
PiezoInferenceResult latestInference;
PiezoSignalQuality latestSignalQuality;

bool featureVectorReady = false;
bool inferenceReady = false;
bool signalWindowAligned = false;

unsigned long featureWindowCount = 0;
unsigned long lastReportMillis = 0;

float modelSourceWindow[PIEZO_WINDOW_SAMPLES];
float alignedWindow[PIEZO_WINDOW_SAMPLES];
float normalizedWindow[PIEZO_WINDOW_SAMPLES];


// ============================================================
// MODEL WINDOW
// ============================================================

void processModelWindow()
{
    const PiezoReading &reading =
        piezo.getReading();

    if (!reading.newFeatureWindowDue)
    {
        return;
    }

    featureVectorReady = false;
    inferenceReady = false;
    signalWindowAligned = false;
    latestSignalQuality =
        PiezoSignalQuality{};

    if (
        !piezo.copyModelSourceWindow(
            modelSourceWindow,
            PIEZO_WINDOW_SAMPLES
        )
    )
    {
        piezo.acknowledgeFeatureWindow();
        return;
    }

    if (
        !signalProcessor.alignWindow(
            modelSourceWindow,
            alignedWindow,
            PIEZO_WINDOW_SAMPLES,
            latestSignalQuality
        )
    )
    {
        piezo.acknowledgeFeatureWindow();
        return;
    }

    signalWindowAligned = true;

    if (!latestSignalQuality.valid)
    {
        piezo.acknowledgeFeatureWindow();
        return;
    }

    if (
        !featureExtractor.robustNormalizeWindow(
            alignedWindow,
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

    featureVectorReady = true;
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
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println(
        "=========================================="
    );
    Serial.println(
        " SafeSeat Piezo Seatbelt Runtime"
    );
    Serial.println(
        " Step 5.7.2 - ML + Main Hub Communication"
    );
    Serial.println(
        "=========================================="
    );

    const bool piezoReady =
        piezo.begin();

    Serial.println(
        piezoReady
            ? "[PIEZO] Acquisition ready."
            : "[PIEZO] ERROR."
    );

    const bool commReady =
        piezoComm.begin();

    Serial.println(
        commReady
            ? "[PIEZO-COMM] UART TX ready on GPIO17 @ 115200."
            : "[PIEZO-COMM] UART initialization failed."
    );

    Serial.println(
        "[ML] 25 Hz raw ADC -> 30 s / 750 samples"
    );
    Serial.println(
        "     -> detrend -> 0.05-1 Hz forward/reverse SOS"
    );
    Serial.println(
        "     -> per-window median/MAD -> 16 features"
    );
    Serial.println(
        "     -> Step 5.7 IF + OCSVM"
    );
    Serial.println(
        "[LINK] Piezo GPIO17 TX -> Main Hub GPIO25 RX + common GND"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    piezo.update();
    processModelWindow();

    const PiezoReading &reading =
        piezo.getReading();

    // Communication is independent from Serial dashboard timing.
    piezoComm.update(
        reading,
        signalWindowAligned,
        latestSignalQuality,
        featureVectorReady,
        inferenceReady,
        latestFeatures,
        latestInference,
        featureWindowCount
    );

    const unsigned long now =
        millis();

    if (
        now - lastReportMillis
        <
        PIEZO_SERIAL_REPORT_INTERVAL_MS
    )
    {
        return;
    }

    lastReportMillis = now;

    Serial.println();
    Serial.println(
        "=========================================="
    );
    Serial.println(
        " SAFESEAT PIEZO ESP32 - STEP 5.7.2"
    );
    Serial.println(
        "=========================================="
    );

    Serial.printf(
        "Sample rate       : %.2f Hz\n",
        reading.actualSamplingRateHz
    );

    Serial.printf(
        "Samples           : %lu\n",
        reading.sampleCount
    );

    Serial.printf(
        "Raw ADC           : %d\n",
        reading.rawADC
    );

    Serial.printf(
        "Aux resp waveform : %.2f\n",
        reading.respirationWave
    );

    if (
        isfinite(
            reading.estimatedRespirationBPM
        )
    )
    {
        Serial.printf(
            "Peak-based RR     : %.1f BPM\n",
            reading.estimatedRespirationBPM
        );
    }
    else
    {
        Serial.println(
            "Peak-based RR     : not ready"
        );
    }

    Serial.printf(
        "No-breath timer   : %.1f sec%s\n",
        reading.noBreathDurationMs / 1000.0f,
        reading.noBreathTimerExceeded
            ? " [AUX TIMER EXCEEDED]"
            : ""
    );

    Serial.printf(
        "30s window        : %u / %u\n",
        reading.windowSamplesAvailable,
        PIEZO_WINDOW_SAMPLES
    );

    Serial.println(
        "------------------------------------------"
    );

    if (signalWindowAligned)
    {
        Serial.printf(
            "Signal quality    : %s\n",
            latestSignalQuality.valid
                ? "VALID"
                : "REJECTED"
        );

        Serial.printf(
            "ADC rail fraction : %.2f %%\n",
            latestSignalQuality.railFraction
            *
            100.0f
        );

        Serial.printf(
            "Aligned std       : %.4f\n",
            latestSignalQuality.alignedStd
        );

        Serial.printf(
            "Aligned range     : %.4f\n",
            latestSignalQuality.alignedMax
            -
            latestSignalQuality.alignedMin
        );

        if (latestSignalQuality.excessiveRailContact)
        {
            Serial.println(
                "Reject reason     : ADC clipping/rail contact"
            );
        }

        if (latestSignalQuality.effectivelyFlat)
        {
            Serial.println(
                "Reject reason     : aligned signal too flat"
            );
        }
    }
    else
    {
        Serial.println(
            "Signal alignment  : waiting for first 30s window"
        );
    }

    Serial.println(
        "------------------------------------------"
    );

    Serial.printf(
        "Feature windows   : %lu\n",
        featureWindowCount
    );

    if (featureVectorReady)
    {
        Serial.printf(
            "Spectral RR       : %.2f BPM\n",
            latestFeatures.respirationBPM
        );

        Serial.printf(
            "Spectral entropy  : %.6f\n",
            latestFeatures.spectralEntropy
        );

        Serial.printf(
            "Autocorr peak     : %.6f\n",
            latestFeatures.autocorrelationPeak
        );
    }

    if (
        inferenceReady
        &&
        latestInference.valid
    )
    {
        Serial.printf(
            "IsolationForest   : %.6f -> %s\n",
            latestInference.isolationForestDecision,
            latestInference.isolationForestAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.printf(
            "One-Class SVM     : %.6f -> %s\n",
            latestInference.oneClassSVMDecision,
            latestInference.oneClassSVMAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.print(
            "Model agreement   : "
        );

        if (latestInference.bothModelsAnomaly)
        {
            Serial.println(
                "STRONG RESPIRATION-PATTERN ANOMALY"
            );
        }
        else if (latestInference.eitherModelAnomaly)
        {
            Serial.println(
                "WEAK RESPIRATION-PATTERN ANOMALY"
            );
        }
        else
        {
            Serial.println(
                "NORMAL RESPIRATION PATTERN"
            );
        }

        Serial.println(
            "Fusion policy     : surrogate evidence; corroboration required"
        );
    }
    else if (
        signalWindowAligned
        &&
        !latestSignalQuality.valid
    )
    {
        Serial.println(
            "Inference         : SKIPPED - poor signal quality"
        );
    }
    else
    {
        Serial.println(
            "Inference         : waiting"
        );
    }

    Serial.println(
        "------------------------------------------"
    );

    Serial.printf(
        "Packets sent      : %lu\n",
        piezoComm.getPacketsSent()
    );

    Serial.println(
        "Link              : GPIO17 TX -> Main GPIO25 RX"
    );

    Serial.println(
        "=========================================="
    );
}
