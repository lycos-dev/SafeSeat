#include "Config.h"
#include "C1001.h"


C1001Sensor c1001;


unsigned long lastPrintTime = 0;


const unsigned long
    PRINT_INTERVAL_MS = 1000;


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
        " SafeSeat Main Hub Runtime"
    );

    Serial.println(
        " Stage 1 - C1001"
    );

    Serial.println(
        "=========================================="
    );


    c1001.begin();
}


void loop()
{
    c1001.update();


    unsigned long now =
        millis();


    if (
        now
        -
        lastPrintTime
        <
        PRINT_INTERVAL_MS
    )
    {
        return;
    }


    lastPrintTime =
        now;


    const C1001Reading&
        r =
            c1001.getReading();


    Serial.println();
    Serial.println(
        "------------------------------------------"
    );


    Serial.print(
        "Status       : "
    );

    Serial.println(
        c1001.getStatusText()
    );


    Serial.print(
        "Present      : "
    );

    Serial.println(
        r.present
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Motion       : "
    );

    Serial.println(
        r.motion
    );


    Serial.print(
        "MoveRange    : "
    );

    Serial.println(
        r.moveRange
    );


    Serial.print(
        "Raw RR       : "
    );

    Serial.println(
        r.rawRespiration
    );


    Serial.print(
        "Raw HR       : "
    );

    Serial.println(
        r.rawHeartRate
    );


    if (
        r.status ==
        C1001Status::WARMING_UP
    )
    {
        Serial.print(
            "Warmup left  : "
        );

        Serial.print(
            r.warmupRemainingSeconds
        );

        Serial.println(
            " s"
        );
    }


    if (
        c1001.hasTrustedVitals()
    )
    {
        Serial.print(
            "Filtered RR  : "
        );

        Serial.println(
            r.filteredRespiration,
            1
        );


        Serial.print(
            "Filtered HR  : "
        );

        Serial.println(
            r.filteredHeartRate,
            1
        );
    }
    else
    {
        Serial.println(
            "Filtered RR  : not ready"
        );

        Serial.println(
            "Filtered HR  : not ready"
        );
    }


    Serial.print(
        "Clean samples: "
    );

    Serial.println(
        r.cleanSampleCount
    );
}