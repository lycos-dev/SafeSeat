#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

#include "C1001.h"
#include "MLX.h"
#include "FSR.h"
#include "MPU.h"


// ============================================================
// SENSOR OBJECTS
// ============================================================

C1001Sensor c1001;

MLXSensor mlx;

FSRSensor fsr;

MPUSensor mpu;


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
        " Stage 4 - Main Sensors"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // SHARED I2C BUS
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting I2C bus..."
    );


    Wire.begin(
        I2C_SDA_PIN,
        I2C_SCL_PIN
    );


    /*
     * 400 kHz significantly reduces I2C transaction time.
     *
     * MLX90614, ADS1115 and MPU6050 share this bus.
     */
    Wire.setClock(
        400000
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


    Serial.println(
        c1001Ready
            ? "[MAIN] C1001 ready."
            : "[MAIN] WARNING: C1001 failed."
    );


    // ========================================================
    // MLX90614
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting MLX90614..."
    );


    bool mlxReady =
        mlx.begin();


    Serial.println(
        mlxReady
            ? "[MAIN] MLX90614 ready."
            : "[MAIN] WARNING: MLX90614 failed."
    );


    // ========================================================
    // MPU6050
    //
    // Initialize before the slower FSR calibration.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting MPU6050..."
    );


    bool mpuReady =
        mpu.begin();


    Serial.println(
        mpuReady
            ? "[MAIN] MPU6050 ready."
            : "[MAIN] WARNING: MPU6050 failed."
    );


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


    Serial.println(
        fsrReady
            ? "[MAIN] FSR array ready."
            : "[MAIN] WARNING: FSR initialization failed."
    );


    // ========================================================
    // COMPLETE
    // ========================================================

    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " MAIN SENSOR INITIALIZATION COMPLETE"
    );

    Serial.println(
        "=========================================="
    );


    Serial.println(
        "C1001    : acquisition active"
    );

    Serial.println(
        "MLX90614 : acquisition active"
    );

    Serial.println(
        "FSR      : acquisition active"
    );

    Serial.println(
        "MPU6050  : acquisition active"
    );


    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // HIGH-RATE MPU
    //
    // Keep this call first.
    // ========================================================

    mpu.update();


    // ========================================================
    // OTHER SENSOR MODULES
    // ========================================================

    c1001.update();


    mlx.update();


    bool occupantPresent =
        c1001.getReading()
            .present;


    fsr.update(
        occupantPresent
    );


    /*
     * Give MPU another opportunity immediately after the
     * potentially slower FSR acquisition.
     */
    mpu.update();


    // ========================================================
    // SERIAL DASHBOARD
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
        c =
            c1001.getReading();


    const MLXReading&
        t =
            mlx.getReading();


    const FSRReading&
        f =
            fsr.getReading();


    const MPUReading&
        m =
            mpu.getReading();


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
        "Presence       : "
    );

    Serial.println(
        c.present
            ? "YES"
            : "NO"
    );


    Serial.print(
        "MoveRange      : "
    );

    Serial.println(
        c.moveRange
    );


    Serial.print(
        "Raw RR / HR    : "
    );

    Serial.print(
        c.rawRespiration
    );

    Serial.print(
        " / "
    );

    Serial.println(
        c.rawHeartRate
    );


    if (
        c.status
        ==
        C1001Status::WARMING_UP
    )
    {
        Serial.print(
            "Warm-up left   : "
        );

        Serial.print(
            c.warmupRemainingSeconds
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
            c.filteredRespiration,
            1
        );

        Serial.println(
            " BPM"
        );


        Serial.print(
            "Filtered HR    : "
        );

        Serial.print(
            c.filteredHeartRate,
            1
        );

        Serial.println(
            " BPM"
        );
    }
    else
    {
        Serial.println(
            "Filtered vitals: not ready"
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


    if (
        mlx.hasValidReading()
    )
    {
        Serial.print(
            "Ambient        : "
        );

        Serial.print(
            t.filteredAmbientC,
            2
        );

        Serial.println(
            " C"
        );


        Serial.print(
            "Object         : "
        );

        Serial.print(
            t.filteredObjectC,
            2
        );

        Serial.println(
            " C"
        );


        Serial.print(
            "Object-Ambient : "
        );

        Serial.print(
            t.objectMinusAmbientC,
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
    // FSR
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
        "Actual Fs      : "
    );

    Serial.print(
        f.actualSamplingRateHz,
        2
    );

    Serial.println(
        " Hz"
    );


    if (
        fsr.hasValidReading()
    )
    {
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
                fsr.getSensorLabel(
                    i
                )
            );

            Serial.print(
                " = "
            );

            Serial.println(
                f.pressure[i],
                1
            );
        }


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
                fsr.getSensorLabel(
                    i
                )
            );

            Serial.print(
                " = "
            );

            Serial.println(
                f.pressure[i],
                1
            );
        }


        Serial.print(
            "Backrest total : "
        );

        Serial.println(
            f.backrestTotal,
            1
        );


        Serial.print(
            "Cushion total  : "
        );

        Serial.println(
            f.cushionTotal,
            1
        );


        Serial.print(
            "Whole total    : "
        );

        Serial.println(
            f.wholeSeatTotal,
            1
        );


        Serial.print(
            "Back L/R       : "
        );

        Serial.println(
            f.backrestLRBalance,
            3
        );


        Serial.print(
            "Cushion L/R    : "
        );

        Serial.println(
            f.cushionLRBalance,
            3
        );


        Serial.print(
            "Back/Cushion   : "
        );

        Serial.println(
            f.backrestToCushionRatio,
            3
        );
    }


    // ========================================================
    // MPU6050
    // ========================================================

    Serial.println();
    Serial.println(
        "-------------- MPU6050 ------------------"
    );


    Serial.print(
        "Status         : "
    );

    Serial.println(
        mpu.getStatusText()
    );


    Serial.print(
        "Samples        : "
    );

    Serial.println(
        m.sampleCount
    );


    Serial.print(
        "Actual Fs      : "
    );

    Serial.print(
        m.actualSamplingRateHz,
        2
    );

    Serial.println(
        " Hz"
    );


    if (
        mpu.hasValidReading()
    )
    {
        Serial.print(
            "Accel X/Y/Z    : "
        );

        Serial.print(
            m.accelX,
            4
        );

        Serial.print(
            " / "
        );

        Serial.print(
            m.accelY,
            4
        );

        Serial.print(
            " / "
        );

        Serial.print(
            m.accelZ,
            4
        );

        Serial.println(
            " g"
        );


        Serial.print(
            "Gyro X/Y/Z     : "
        );

        Serial.print(
            m.gyroX,
            3
        );

        Serial.print(
            " / "
        );

        Serial.print(
            m.gyroY,
            3
        );

        Serial.print(
            " / "
        );

        Serial.print(
            m.gyroZ,
            3
        );

        Serial.println(
            " deg/s"
        );


        Serial.print(
            "Accel magnitude: "
        );

        Serial.print(
            m.accelMagnitude,
            4
        );

        Serial.println(
            " g"
        );


        Serial.print(
            "Gyro magnitude : "
        );

        Serial.print(
            m.gyroMagnitude,
            3
        );

        Serial.println(
            " deg/s"
        );


        Serial.print(
            "Dynamic accel  : "
        );

        Serial.print(
            m.dynamicAcceleration,
            4
        );

        Serial.println(
            " g"
        );
    }


    // ========================================================
    // UPCOMING SYSTEM LAYERS
    // ========================================================

    Serial.println();
    Serial.println(
        "------------------------------------------"
    );


    Serial.println(
        "Piezo          : separate ESP32"
    );


    Serial.println(
        "ML inference   : next phase"
    );


    Serial.println(
        "Fusion         : after inference"
    );


    Serial.println(
        "Camera         : separate ESP32-CAM"
    );


    Serial.println(
        "=========================================="
    );
}