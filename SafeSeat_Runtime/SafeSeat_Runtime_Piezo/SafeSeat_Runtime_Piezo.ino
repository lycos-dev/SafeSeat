#include <Arduino.h>

#include "Config.h"
#include "PiezoSensor.h"


// ============================================================
// SENSOR
// ============================================================

PiezoSensor piezo;


// ============================================================
// SERIAL REPORTING
// ============================================================

unsigned long lastReportMillis =
    0;


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
        " Stage 1 - Piezo Acquisition"
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
            "[MAIN] Piezo acquisition ready."
        );
    }
    else
    {
        Serial.println(
            "[MAIN] ERROR: Piezo initialization failed."
        );
    }


    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    piezo.update();


    unsigned long now =
        millis();


    if (
        now
        -
        lastReportMillis
        <
        PIEZO_SERIAL_REPORT_INTERVAL_MS
    )
    {
        return;
    }


    lastReportMillis =
        now;


    const PiezoReading&
        p =
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


    Serial.print(
        "Status          : "
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
        "Sample rate     : "
    );

    Serial.print(
        p.actualSamplingRateHz,
        2
    );

    Serial.println(
        " Hz"
    );


    Serial.print(
        "Sample count    : "
    );

    Serial.println(
        p.sampleCount
    );


    Serial.println();
    Serial.println(
        "Signal:"
    );


    Serial.print(
        "  Raw ADC       : "
    );

    Serial.println(
        p.rawADC
    );


    Serial.print(
        "  Filtered      : "
    );

    Serial.println(
        p.filteredSignal,
        2
    );


    Serial.print(
        "  Baseline      : "
    );

    Serial.println(
        p.baseline,
        2
    );


    Serial.print(
        "  Resp waveform : "
    );

    Serial.println(
        p.respirationWave,
        2
    );


    Serial.println();
    Serial.println(
        "Breathing context:"
    );


    Serial.print(
        "  Total breaths : "
    );

    Serial.println(
        p.totalBreaths
    );


    Serial.print(
        "  Estimated RR  : "
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
        "  No-breath time: "
    );

    Serial.print(
        p.noBreathDurationMs
        /
        1000.0f,
        1
    );

    Serial.println(
        " sec"
    );


    Serial.print(
        "  15s timer     : "
    );

    Serial.println(
        p.noBreathTimerExceeded
            ? "EXCEEDED"
            : "NO"
    );


    Serial.println();
    Serial.println(
        "ML window:"
    );


    Serial.print(
        "  Samples       : "
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
        "  30s ready     : "
    );

    Serial.println(
        p.fullWindowReady
            ? "YES"
            : "NO"
    );


    Serial.print(
        "  Feature due   : "
    );

    Serial.println(
        p.newFeatureWindowDue
            ? "YES"
            : "NO"
    );


    Serial.println();
    Serial.println(
        "Inference       : not integrated yet"
    );

    Serial.println(
        "Communication   : not integrated yet"
    );


    Serial.println(
        "=========================================="
    );
}