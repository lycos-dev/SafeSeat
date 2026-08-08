#include "Config.h"
#include "C1001.h"

C1001Sensor c1001;

unsigned long lastSample = 0;
unsigned long lastReport = 0;

void setup() {

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        " SafeSeat Runtime - Stage 1"
    );
    Serial.println(
        "================================"
    );

    Serial.println(
        "Initializing C1001..."
    );

    if (
        c1001.begin()
    ) {

        Serial.println(
            "C1001 initialized."
        );

    } else {

        Serial.println(
            "ERROR: C1001 initialization failed."
        );
    }
}

void loop() {

    unsigned long now =
        millis();

    if (
        now - lastSample
        >= SENSOR_SAMPLE_INTERVAL_MS
    ) {

        lastSample = now;

        c1001.update();
    }

    if (
        now - lastReport
        >= SERIAL_REPORT_INTERVAL_MS
    ) {

        lastReport = now;

        const C1001Reading&
            reading =
                c1001.getReading();

        Serial.println();
        Serial.println(
            "----- C1001 -----"
        );

        Serial.print(
            "Connected: "
        );

        Serial.println(
            reading.connected
                ? "YES"
                : "NO"
        );

        Serial.print(
            "Presence: "
        );

        Serial.println(
            reading.present
                ? "YES"
                : "NO"
        );

        Serial.print(
            "Warm-up: "
        );

        Serial.println(
            reading.warmedUp
                ? "DONE"
                : "WAITING"
        );

        Serial.print(
            "Raw RR: "
        );

        Serial.println(
            reading.respirationRaw
        );

        Serial.print(
            "Filtered RR: "
        );

        Serial.println(
            reading.respirationFiltered
        );

        Serial.print(
            "Raw HR: "
        );

        Serial.println(
            reading.heartRateRaw
        );

        Serial.print(
            "Filtered HR: "
        );

        Serial.println(
            reading.heartRateFiltered
        );

        Serial.print(
            "MoveRange: "
        );

        Serial.println(
            reading.moveRange
        );
    }
}