#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "C1001.h"
#include "MLX.h"


// ============================================================
// SENSOR OBJECTS
// ============================================================

C1001Sensor c1001;
MLXSensor mlx;


// ============================================================
// SERIAL REPORTING
// ============================================================

unsigned long lastPrintTime = 0;

constexpr unsigned long PRINT_INTERVAL_MS = 1000UL;


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
        " SafeSeat Main Hub Runtime"
    );

    Serial.println(
        " Stage 2 - C1001 + MLX90614"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // SHARED I2C BUS
    //
    // MLX90614
    // ADS1115 #1
    // ADS1115 #2
    // MPU6050
    //
    // all share:
    //
    // SDA = GPIO21
    // SCL = GPIO22
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting I2C bus..."
    );


    Wire.begin(
        I2C_SDA_PIN,
        I2C_SCL_PIN
    );


    Serial.println(
        "[MAIN] I2C bus ready."
    );


    // ========================================================
    // C1001
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting C1001..."
    );


    bool c1001Ready =
        c1001.begin();


    if (c1001Ready)
    {
        Serial.println(
            "[MAIN] C1001 ready."
        );
    }
    else
    {
        Serial.println(
            "[MAIN] WARNING: C1001 failed to initialize."
        );
    }


    // ========================================================
    // MLX90614
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting MLX90614..."
    );


    bool mlxReady =
        mlx.begin();


    if (mlxReady)
    {
        Serial.println(
            "[MAIN] MLX90614 ready."
        );
    }
    else
    {
        Serial.println(
            "[MAIN] WARNING: MLX90614 failed to initialize."
        );
    }


    // ========================================================
    // STAGE 2 STATUS
    // ========================================================

    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " Stage 2 initialization complete"
    );

    Serial.println(
        "=========================================="
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // SENSOR UPDATES
    //
    // Each sensor module controls its own sampling interval.
    // Calling update() frequently here does NOT mean the
    // hardware is read on every loop iteration.
    // ========================================================

    c1001.update();

    mlx.update();


    // ========================================================
    // SERIAL DASHBOARD
    // ========================================================

    unsigned long now =
        millis();


    if (
        now - lastPrintTime
        < PRINT_INTERVAL_MS
    )
    {
        return;
    }


    lastPrintTime =
        now;


    const C1001Reading&
        c1001Reading =
            c1001.getReading();


    const MLXReading&
        mlxReading =
            mlx.getReading();


    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " SAFESEAT MAIN HUB"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // C1001 OUTPUT
    // ========================================================

    Serial.println();
    Serial.println(
        "--------------- C1001 -------------------"
    );


    Serial.print(
        "Status         : "
    );

    Serial.println(
        c1001.getStatusText()
    );


    Serial.print(
        "Connected      : "
    );

    Serial.println(
        c1001Reading.connected
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Presence       : "
    );

    Serial.println(
        c1001Reading.present
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Motion         : "
    );

    Serial.println(
        c1001Reading.motion
    );


    Serial.print(
        "MoveRange      : "
    );

    Serial.println(
        c1001Reading.moveRange
    );


    Serial.print(
        "Raw RR         : "
    );

    Serial.print(
        c1001Reading.rawRespiration
    );

    Serial.println(
        " BPM"
    );


    Serial.print(
        "Raw HR         : "
    );

    Serial.print(
        c1001Reading.rawHeartRate
    );

    Serial.println(
        " BPM"
    );


    Serial.print(
        "RR valid       : "
    );

    Serial.println(
        c1001Reading.validRespiration
            ? "YES"
            : "NO"
    );


    Serial.print(
        "HR valid       : "
    );

    Serial.println(
        c1001Reading.validHeartRate
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Warm-up        : "
    );

    Serial.println(
        c1001Reading.warmedUp
            ? "DONE"
            : "NOT DONE"
    );


    if (
        c1001Reading.status
        ==
        C1001Status::WARMING_UP
    )
    {
        Serial.print(
            "Warm-up left   : "
        );

        Serial.print(
            c1001Reading
                .warmupRemainingSeconds
        );

        Serial.println(
            " s"
        );
    }


    Serial.print(
        "Clean samples  : "
    );

    Serial.println(
        c1001Reading.cleanSampleCount
    );


    Serial.print(
        "Motion artifact: "
    );

    Serial.println(
        c1001Reading
            .motionArtifactActive
            ? "YES"
            : "NO"
    );


    if (
        c1001Reading
            .motionArtifactActive
    )
    {
        Serial.print(
            "Recovery count : "
        );

        Serial.println(
            c1001Reading
                .recoveryStableCount
        );
    }


    if (
        c1001.hasTrustedVitals()
    )
    {
        Serial.print(
            "Median RR      : "
        );

        Serial.println(
            c1001Reading
                .medianRespiration
        );


        Serial.print(
            "Median HR      : "
        );

        Serial.println(
            c1001Reading
                .medianHeartRate
        );


        Serial.print(
            "Filtered RR    : "
        );

        Serial.print(
            c1001Reading
                .filteredRespiration,
            1
        );

        Serial.println(
            " BPM"
        );


        Serial.print(
            "Filtered HR    : "
        );

        Serial.print(
            c1001Reading
                .filteredHeartRate,
            1
        );

        Serial.println(
            " BPM"
        );
    }
    else
    {
        Serial.println(
            "Filtered RR    : not ready"
        );

        Serial.println(
            "Filtered HR    : not ready"
        );
    }


    // ========================================================
    // MLX90614 OUTPUT
    // ========================================================

    Serial.println();
    Serial.println(
        "-------------- MLX90614 -----------------"
    );


    Serial.print(
        "Status         : "
    );

    Serial.println(
        mlx.getStatusText()
    );


    Serial.print(
        "Connected      : "
    );

    Serial.println(
        mlxReading.connected
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Valid reading  : "
    );

    Serial.println(
        mlxReading.valid
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Raw ambient    : "
    );

    if (
        isfinite(
            mlxReading.rawAmbientC
        )
    )
    {
        Serial.print(
            mlxReading.rawAmbientC,
            2
        );

        Serial.println(
            " C"
        );
    }
    else
    {
        Serial.println(
            "N/A"
        );
    }


    Serial.print(
        "Raw object     : "
    );

    if (
        isfinite(
            mlxReading.rawObjectC
        )
    )
    {
        Serial.print(
            mlxReading.rawObjectC,
            2
        );

        Serial.println(
            " C"
        );
    }
    else
    {
        Serial.println(
            "N/A"
        );
    }


    if (
        mlx.hasValidReading()
    )
    {
        Serial.print(
            "Ambient filt.  : "
        );

        Serial.print(
            mlxReading
                .filteredAmbientC,
            2
        );

        Serial.println(
            " C"
        );


        Serial.print(
            "Object filt.   : "
        );

        Serial.print(
            mlxReading
                .filteredObjectC,
            2
        );

        Serial.println(
            " C"
        );


        Serial.print(
            "Object-Ambient : "
        );

        Serial.print(
            mlxReading
                .objectMinusAmbientC,
            2
        );

        Serial.println(
            " C"
        );
    }
    else
    {
        Serial.println(
            "Ambient filt.  : not ready"
        );

        Serial.println(
            "Object filt.   : not ready"
        );

        Serial.println(
            "Object-Ambient : not ready"
        );
    }


    // ========================================================
    // CURRENT STAGE
    // ========================================================

    Serial.println();
    Serial.println(
        "------------------------------------------"
    );

    Serial.println(
        "FSR            : not integrated yet"
    );

    Serial.println(
        "MPU6050        : not integrated yet"
    );

    Serial.println(
        "Piezo          : separate ESP32"
    );

    Serial.println(
        "Fusion         : not active yet"
    );

    Serial.println(
        "Camera         : not active yet"
    );


    Serial.println(
        "=========================================="
    );
}