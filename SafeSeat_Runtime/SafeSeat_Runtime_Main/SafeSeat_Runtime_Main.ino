#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

#include "C1001.h"
#include "MLX.h"
#include "FSR.h"


// ============================================================
// SENSOR OBJECTS
// ============================================================

C1001Sensor c1001;

MLXSensor mlx;

FSRSensor fsr;


// ============================================================
// SERIAL REPORTING
// ============================================================

unsigned long lastPrintTime =
    0;


constexpr unsigned long
    PRINT_INTERVAL_MS =
        1000UL;


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
        " SafeSeat Main Hub Runtime"
    );

    Serial.println(
        " Stage 3 - C1001 + MLX90614 + FSR"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // SHARED I2C
    //
    // GPIO21 SDA
    // GPIO22 SCL
    //
    // MLX90614
    // ADS1115 #1
    // ADS1115 #2
    // MPU6050 (later)
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


    if (
        c1001Ready
    )
    {
        Serial.println(
            "[MAIN] C1001 ready."
        );
    }
    else
    {
        Serial.println(
            "[MAIN] WARNING: C1001 failed."
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


    if (
        mlxReady
    )
    {
        Serial.println(
            "[MAIN] MLX90614 ready."
        );
    }
    else
    {
        Serial.println(
            "[MAIN] WARNING: MLX90614 failed."
        );
    }


    // ========================================================
    // FSR
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting FSR array..."
    );


    Serial.println(
        "[MAIN] IMPORTANT: keep seat empty for calibration."
    );


    bool fsrReady =
        fsr.begin();


    if (
        fsrReady
    )
    {
        Serial.println(
            "[MAIN] FSR array ready."
        );
    }
    else
    {
        Serial.println(
            "[MAIN] WARNING: FSR initialization failed."
        );
    }


    // ========================================================
    // STAGE STATUS
    // ========================================================

    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " Stage 3 initialization complete"
    );

    Serial.println(
        "=========================================="
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // SENSOR UPDATES
    // ========================================================

    c1001.update();


    mlx.update();


    /*
     * C1001 presence is currently used ONLY to protect
     * empty-seat FSR baseline adaptation.
     *
     * It is not an FSR model label.
     */

    bool occupantPresent =
        c1001.getReading().present;


    fsr.update(
        occupantPresent
    );


    // ========================================================
    // SERIAL REPORT
    // ========================================================

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
        c1001Reading =
            c1001.getReading();


    const MLXReading&
        mlxReading =
            mlx.getReading();


    const FSRReading&
        fsrReading =
            fsr.getReading();


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
    // C1001
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


    if (
        c1001.hasTrustedVitals()
    )
    {
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
    // MLX90614
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
            "Temperature    : not ready"
        );
    }


    // ========================================================
    // FSR ARRAY
    // ========================================================

    Serial.println();
    Serial.println(
        "---------------- FSR --------------------"
    );


    Serial.print(
        "Status         : "
    );

    Serial.println(
        fsr.getStatusText()
    );


    Serial.print(
        "Connected      : "
    );

    Serial.println(
        fsrReading.connected
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Calibrated     : "
    );

    Serial.println(
        fsrReading.calibrated
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Actual Fs      : "
    );

    Serial.print(
        fsrReading.actualSamplingRateHz,
        2
    );

    Serial.println(
        " Hz"
    );


    if (
        fsr.hasValidReading()
    )
    {
        Serial.println();
        Serial.println(
            "Backrest:"
        );


        for (
            int i = BACKREST_FSR1;
            i <= BACKREST_FSR6;
            i++
        )
        {
            Serial.print(
                "  "
            );

            Serial.print(
                fsr.getSensorLabel(i)
            );

            Serial.print(
                " = "
            );

            Serial.println(
                fsrReading.pressure[i],
                1
            );
        }


        Serial.println();
        Serial.println(
            "Cushion:"
        );


        for (
            int i = CUSHION_FSR1;
            i <= CUSHION_FSR3;
            i++
        )
        {
            Serial.print(
                "  "
            );

            Serial.print(
                fsr.getSensorLabel(i)
            );

            Serial.print(
                " = "
            );

            Serial.println(
                fsrReading.pressure[i],
                1
            );
        }


        Serial.println();
        Serial.println(
            "FSR distribution:"
        );


        Serial.print(
            "  Backrest total       : "
        );

        Serial.println(
            fsrReading.backrestTotal,
            1
        );


        Serial.print(
            "  Cushion total        : "
        );

        Serial.println(
            fsrReading.cushionTotal,
            1
        );


        Serial.print(
            "  Whole seat total     : "
        );

        Serial.println(
            fsrReading.wholeSeatTotal,
            1
        );


        Serial.print(
            "  Backrest L/R balance : "
        );

        Serial.println(
            fsrReading
                .backrestLRBalance,
            3
        );


        Serial.print(
            "  Cushion L/R balance  : "
        );

        Serial.println(
            fsrReading
                .cushionLRBalance,
            3
        );


        Serial.print(
            "  Cushion center ratio : "
        );

        Serial.println(
            fsrReading
                .cushionCenterRatio,
            3
        );


        Serial.print(
            "  Back/Cushion ratio   : "
        );

        Serial.println(
            fsrReading
                .backrestToCushionRatio,
            3
        );


        Serial.print(
            "  Upper back           : "
        );

        Serial.println(
            fsrReading
                .backrestUpperTotal,
            1
        );


        Serial.print(
            "  Middle back          : "
        );

        Serial.println(
            fsrReading
                .backrestMiddleTotal,
            1
        );


        Serial.print(
            "  Lower back           : "
        );

        Serial.println(
            fsrReading
                .backrestLowerTotal,
            1
        );
    }
    else
    {
        Serial.println(
            "Pressure data  : not ready"
        );
    }


    // ========================================================
    // NOT YET INTEGRATED
    // ========================================================

    Serial.println();
    Serial.println(
        "------------------------------------------"
    );


    Serial.println(
        "MPU6050        : not integrated yet"
    );


    Serial.println(
        "Piezo          : separate ESP32"
    );


    Serial.println(
        "ML inference   : not active yet"
    );


    Serial.println(
        "Fusion         : not active yet"
    );


    Serial.println(
        "Camera         : separate ESP32-CAM"
    );


    Serial.println(
        "=========================================="
    );
}