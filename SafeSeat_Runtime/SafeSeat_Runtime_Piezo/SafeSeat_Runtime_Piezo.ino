#include <Arduino.h>

#include "Config.h"
#include "PiezoSensor.h"
#include "PiezoComm.h"

// ============================================================
// SAFESEAT PIEZO / SEATBELT ESP32
// STEP 5.9.8.2 - DETERMINISTIC RESPIRATION SUPPORT
//
// PVDF @ 25 Hz
// -> EMA smoothing
// -> slow baseline tracking
// -> centered mechanical respiration waveform
// -> conservative breath-event tracking
// -> ESP-NOW support packet
// -> Main Hub Fusion
//
// NO Isolation Forest / One-Class SVM is deployed on this node.
// Piezo is auxiliary respiration corroboration only.
// ============================================================

PiezoSensor piezo;
PiezoComm piezoComm;

unsigned long lastReportMillis = 0;

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
        " SafeSeat Piezo Seatbelt Runtime"
    );
    Serial.println(
        " Step 5.9.8.2 - Deterministic + ESP-NOW"
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
            ? "[PIEZO-COMM] ESP-NOW ready; searching for Main Hub beacon."
            : "[PIEZO-COMM] ESP-NOW initialization failed."
    );

    Serial.println(
        "[PIPELINE] raw ADC -> EMA -> baseline removal -> mechanical breath events"
    );

    Serial.println(
        "[POLICY] Piezo cannot independently create WARNING/EMERGENCY."
    );

    Serial.println(
        "[LINK] ESP-NOW wireless; no Piezo/Main signal wire required."
    );
}

void loop()
{
    piezo.update();

    const PiezoReading &reading =
        piezo.getReading();

    // Communication remains independent from dashboard timing.
    piezoComm.update(
        reading
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

    lastReportMillis =
        now;

    Serial.println();
    Serial.println(
        "=========================================="
    );
    Serial.println(
        " SAFESEAT PIEZO - DETERMINISTIC SUPPORT"
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
        "Filtered signal   : %.2f\n",
        reading.filteredSignal
    );

    Serial.printf(
        "Slow baseline     : %.2f\n",
        reading.baseline
    );

    Serial.printf(
        "Resp waveform     : %.2f\n",
        reading.respirationWave
    );

    Serial.print(
        "Signal usable     : "
    );
    Serial.println(
        reading.signalUsable
            ? "YES"
            : "STARTING / INVALID"
    );

    Serial.print(
        "Breath tracking   : "
    );
    Serial.println(
        reading.breathTrackingReady
            ? "READY"
            : "LEARNING INITIAL EVENTS"
    );

    Serial.printf(
        "Breath events     : %lu\n",
        reading.totalBreaths
    );

    Serial.print(
        "Estimated RR      : "
    );

    if (
        isfinite(
            reading.estimatedRespirationBPM
        )
    )
    {
        Serial.print(
            reading.estimatedRespirationBPM,
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

    Serial.printf(
        "Last event age    : %.1f sec\n",
        reading.noBreathDurationMs
        /
        1000.0f
    );

    Serial.print(
        "No-breath support : "
    );

    Serial.println(
        reading.noBreathTimerExceeded
            ? "ACTIVE (CORROBORATION ONLY)"
            : "NO"
    );

    Serial.println(
        "------------------------------------------"
    );

    Serial.printf(
        "Packets sent      : %lu\n",
        piezoComm.getPacketsSent()
    );

    Serial.printf(
        "Send errors       : %lu\n",
        piezoComm.getSendErrors()
    );

    Serial.printf(
        "Hub beacons       : %lu\n",
        piezoComm.getHubBeaconsReceived()
    );

    Serial.print(
        "ESP-NOW link      : "
    );

    Serial.println(
        piezoComm.isHubLocked()
            ? "LOCKED"
            : "SCANNING"
    );

    Serial.printf(
        "Wi-Fi channel     : %u\n",
        piezoComm.getChannel()
    );

    if (
        piezoComm.isHubLocked()
    )
    {
        Serial.printf(
            "Beacon age        : %lu ms\n",
            piezoComm.getBeaconAgeMillis()
        );
    }

    Serial.println(
        "Fusion role       : secondary respiration support only"
    );

    Serial.println(
        "=========================================="
    );
}
