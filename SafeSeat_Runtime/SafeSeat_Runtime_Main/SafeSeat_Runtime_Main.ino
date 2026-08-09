#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

#include "C1001.h"
#include "C1001ML.h"
#include "MLX.h"
#include "MLXML.h"
#include "MLXContext.h"
#include "FSR.h"
#include "FSRML.h"
#include "MPU.h"
#include "MPUML.h"
#include "Fusion.h"
#include "PiezoComm.h"


// ============================================================
// SENSOR OBJECTS
// ============================================================

C1001Sensor c1001;
C1001ML c1001ML;
MLXSensor mlx;
MLXML mlxML;
MLXContext mlxContext;
FSRSensor fsr;
FSRML fsrML;
MPUSensor mpu;
MPUML mpuML;
FusionEngine fusion;
PiezoComm piezoComm;


// ============================================================
// INITIALIZATION STATE
//
// These flags describe what actually happened during begin().
// We do NOT print "acquisition active" for a module that failed
// initialization.
// ============================================================

bool c1001Initialized = false;
bool mlxInitialized = false;
bool fsrInitialized = false;
bool mpuInitialized = false;
bool piezoCommInitialized = false;


// ============================================================
// C1001 BACKGROUND TASK
//
// DFRobot C1001 library queries are comparatively slow and
// were starving the I2C acquisition loop.
//
// ESP32 Arduino loop() normally runs on core 1. C1001 UART work
// is moved to core 0 so FSR + MPU timing can continue.
//
// C1001's own update() still enforces its existing 1 Hz sample
// interval and all existing warm-up/filter logic is unchanged.
// ============================================================

TaskHandle_t c1001TaskHandle =
    nullptr;


void c1001Task(
    void *parameter
)
{
    (void) parameter;


    while (
        true
    )
    {
        if (
            c1001Initialized
        )
        {
            c1001.update();
        }


        vTaskDelay(
            pdMS_TO_TICKS(
                10
            )
        );
    }
}


// ============================================================
// SERIAL REPORTING
// ============================================================

unsigned long lastPrintTime = 0;


// ============================================================
// HELPERS
// ============================================================

const char* readyText(
    bool ready
)
{
    return ready
        ? "READY"
        : "FAILED / UNAVAILABLE";
}


FusionSensorHealth mapSensorHealth(
    bool available,
    bool connected,
    bool valid,
    bool warmingUp
)
{
    if (
        !available
        ||
        !connected
    )
    {
        return FusionSensorHealth::UNAVAILABLE;
    }

    if (
        warmingUp
    )
    {
        return FusionSensorHealth::WARMING_UP;
    }

    if (
        !valid
    )
    {
        return FusionSensorHealth::DEGRADED;
    }

    return FusionSensorHealth::VALID;
}


void printInitializationSummary()
{
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

    Serial.print(
        "C1001    : "
    );
    Serial.println(
        readyText(
            c1001Initialized
        )
    );

    Serial.print(
        "MLX90614 : "
    );
    Serial.println(
        readyText(
            mlxInitialized
        )
    );

    Serial.print(
        "FSR      : "
    );
    Serial.println(
        readyText(
            fsrInitialized
        )
    );

    Serial.print(
        "MPU6050  : "
    );
    Serial.println(
        readyText(
            mpuInitialized
        )
    );

    Serial.print(
        "PiezoLink: "
    );
    Serial.println(
        readyText(
            piezoCommInitialized
        )
    );

    Serial.println();
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
        " SafeSeat Main Hub Runtime"
    );

    Serial.println(
        " Step 5.7.2 - Piezo Communication + Full Sensor Fusion"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // ONE SHARED I2C BUS
    //
    // This mirrors the proven combined sketch:
    //
    //     Wire.begin(21, 22);
    //
    // All I2C modules use this already-running bus.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting shared I2C bus..."
    );

    Wire.begin(
        I2C_SDA_PIN,
        I2C_SCL_PIN
    );

    Wire.setClock(
        I2C_CLOCK_HZ
    );

    Serial.print(
        "[MAIN] I2C ready on SDA="
    );

    Serial.print(
        I2C_SDA_PIN
    );

    Serial.print(
        ", SCL="
    );

    Serial.print(
        I2C_SCL_PIN
    );

    Serial.print(
        " @ "
    );

    Serial.print(
        I2C_CLOCK_HZ
        /
        1000UL
    );

    Serial.println(
        " kHz."
    );


    // ========================================================
    // ONE SHARED ESP32 ADC CONFIGURATION
    //
    // Same setup used by the proven combined sketch.
    // ========================================================

    Serial.println(
        "[MAIN] Configuring shared ESP32 ADC..."
    );

    analogReadResolution(
        MAIN_ADC_RESOLUTION_BITS
    );

    analogSetAttenuation(
        ADC_11db
    );

    pinMode(
        CUSHION_FSR3_PIN,
        INPUT
    );

    Serial.println(
        "[MAIN] ADC ready: 12-bit, 11 dB attenuation."
    );


    // ========================================================
    // C1001
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting C1001..."
    );

    c1001Initialized =
        c1001.begin();

    Serial.println(
        c1001Initialized
            ? "[MAIN] C1001 ready."
            : "[MAIN] WARNING: C1001 failed."
    );


    // ========================================================
    // C1001 EMBEDDED ML
    //
    // Model parameters are compiled into flash. The ML layer
    // waits for a warmed-up occupant session and then builds the
    // exact 30-second / 15-second-stride feature windows used
    // during training.
    // ========================================================

    c1001ML.begin();


    // ========================================================
    // MLX90614
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting MLX90614..."
    );

    mlxInitialized =
        mlx.begin();

    Serial.println(
        mlxInitialized
            ? "[MAIN] MLX90614 ready."
            : "[MAIN] WARNING: MLX90614 failed."
    );


    // ========================================================
    // MLX90614 EMBEDDED ML
    //
    // Uses the raw object-temperature stream at 4 Hz to match
    // the WESAD TEMP training feature cadence. Ambient
    // temperature remains separate runtime context.
    // ========================================================

    mlxML.begin();

    // Step 5.4.5 deployment-safe MLX context evidence.
    // The WESAD model above remains diagnostic-only; Fusion
    // consumes mlxContext instead.
    mlxContext.begin();


    // ========================================================
    // MPU6050
    //
    // Kept before the slower FSR empty-seat calibration.
    // The MPU implementation itself is NOT changed in Step 1.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting MPU6050..."
    );

    mpuInitialized =
        mpu.begin();

    Serial.println(
        mpuInitialized
            ? "[MAIN] MPU6050 ready."
            : "[MAIN] WARNING: MPU6050 failed."
    );


    // ========================================================
    // FSR
    //
    // The FSR implementation itself is NOT changed in Step 1.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting FSR array..."
    );

    Serial.println(
        "[MAIN] IMPORTANT: keep seat empty for calibration."
    );

    fsrInitialized =
        fsr.begin();

    Serial.println(
        fsrInitialized
            ? "[MAIN] FSR array ready."
            : "[MAIN] WARNING: FSR initialization failed."
    );


    // ========================================================
    // FSR EMBEDDED ML - STEP 5.5
    //
    // Consumes completed calibrated FSR frames. Each frame is
    // normalized to nine pressure shares; absolute pressure
    // magnitude is intentionally excluded from the model.
    // ========================================================

    fsrML.begin();


    // ========================================================
    // RUNTIME TIMING STARTS NOW
    //
    // FSR calibration intentionally blocks setup for several
    // seconds. Reset MPU rate diagnostics so those startup
    // seconds do not pollute its reported runtime Fs.
    // ========================================================

    if (
        mpuInitialized
    )
    {
        mpu.resetSamplingDiagnostics();
    }


    // ========================================================
    // MPU6050 EMBEDDED ML - STEP 5.6
    //
    // Road Data training used calibrated acceleration in m/s^2
    // with stationary offsets removed and gyro in rad/s.
    // MPUML learns a one-second stationary startup baseline from
    // this real MPU6050 before collecting model windows.
    //
    // The model result is road/vehicle-motion context only.
    // It is never counted as occupant medical anomaly evidence.
    // ========================================================

    mpuML.begin();


    // ========================================================
    // PIEZO REMOTE UART - STEP 5.7.2
    //
    // Receive-only link from the separate seatbelt ESP32.
    // Wiring: Piezo TX GPIO17 -> Main RX GPIO25 + common GND.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting Piezo evidence link..."
    );

    piezoCommInitialized =
        piezoComm.begin();

    Serial.println(
        piezoCommInitialized
            ? "[MAIN] Piezo UART RX ready on GPIO25 @ 115200."
            : "[MAIN] WARNING: Piezo UART link failed."
    );


    // ========================================================
    // START C1001 UART TASK ON CORE 0
    // ========================================================

    if (
        c1001Initialized
    )
    {
        BaseType_t taskResult =
            xTaskCreatePinnedToCore(
                c1001Task,
                "SafeSeat_C1001",
                4096,
                nullptr,
                1,
                &c1001TaskHandle,
                0
            );


        if (
            taskResult
            ==
            pdPASS
        )
        {
            Serial.println(
                "[MAIN] C1001 background task started on core 0."
            );
        }
        else
        {
            Serial.println(
                "[MAIN] WARNING: C1001 background task creation failed."
            );
        }
    }


    printInitializationSummary();

    fusion.begin();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // UPDATE ONLY MODULES THAT ACTUALLY INITIALIZED
    //
    // Failed/disconnected sensors are not treated as normal
    // data. Their reading structures remain available for the
    // dashboard and, later, Fusion validity gating.
    // ========================================================

    if (
        piezoCommInitialized
    )
    {
        piezoComm.update();
    }

    if (
        mpuInitialized
    )
    {
        mpu.update();
    }

    // C1001 updates on core 0 in c1001Task().
    // Do not run its blocking UART queries on the I2C loop.

    if (
        mlxInitialized
    )
    {
        mlx.update();
    }

    bool occupantPresent =
        c1001Initialized
        &&
        c1001.getReading().present;

    if (
        fsrInitialized
    )
    {
        fsr.update(
            occupantPresent
        );
    }

    // Give the high-rate MPU another opportunity after the
    // potentially slower I2C/FSR work.
    if (
        mpuInitialized
    )
    {
        mpu.update();
    }


    // Take one C1001 value snapshot for this main-loop pass.
    // C1001 is updated on core 0; using a local copy avoids
    // repeatedly dereferencing changing fields while building
    // the ML/Fusion inputs.
    const C1001Reading c =
        c1001.getReading();

    c1001ML.update(
        c
    );

    const C1001MLReading &cml =
        c1001ML.getReading();

    const MLXReading &t =
        mlx.getReading();

    mlxML.update(
        t
    );

    const MLXMLReading &tml =
        mlxML.getReading();

    mlxContext.update(
        t
    );

    const MLXContextReading &tx =
        mlxContext.getReading();

    const FSRReading &f =
        fsr.getReading();

    fsrML.update(
        f,
        c.present
    );

    const FSRMLReading &fml =
        fsrML.getReading();

    const MPUReading &m =
        mpu.getReading();

    mpuML.update(
        m
    );

    const MPUMLReading &mml =
        mpuML.getReading();


    FusionInput fusionInput;
    fusionInput.timestampMillis =
        millis();

    fusionInput.c1001.health =
        mapSensorHealth(
            c1001Initialized,
            c.connected,
            c.trustedVitalsAvailable,
            c.status
            ==
            C1001Status::WARMING_UP
        );

    fusionInput.c1001.reading =
        c;

    // C1001 model evidence is produced by the sensor-specific
    // C1001 ML pipeline. Fusion receives only the results.
    fusionInput.c1001.model.available =
        cml.modelAvailable;

    fusionInput.c1001.model.valid =
        cml.valid;

    fusionInput.c1001.model.isolationForestAnomaly =
        cml.isolationForestAnomaly;

    fusionInput.c1001.model.oneClassSVMAnomaly =
        cml.oneClassSVMAnomaly;

    fusionInput.c1001.model.bothModelsAnomaly =
        cml.bothModelsAnomaly;

    fusionInput.c1001.model.eitherModelAnomaly =
        cml.eitherModelAnomaly;

    fusionInput.c1001.model.isolationForestScore =
        cml.isolationForestDecision;

    fusionInput.c1001.model.oneClassSVMScore =
        cml.oneClassSVMDecision;

    fusionInput.c1001.model.confidence =
        cml.valid
            ? 1.0f
            : 0.0f;

    fusionInput.mlx.health =
        mapSensorHealth(
            mlxInitialized,
            t.connected,
            t.valid,
            t.status
            ==
            MLXStatus::INITIALIZING
        );

    fusionInput.mlx.reading =
        t;

    fusionInput.mlx.context =
        tx;

    // MLX WESAD model evidence is retained for diagnostics.
    // Step 5.4.5 Fusion intentionally ignores this model because
    // the contact-E4 -> non-contact-MLX domain mismatch was
    // demonstrated in Step 5.4.3.
    //
    // MLX model evidence is produced by the sensor-specific
    // MLX ML pipeline. Ambient temperature is not model input.
    fusionInput.mlx.model.available =
        tml.modelAvailable;

    fusionInput.mlx.model.valid =
        tml.valid;

    fusionInput.mlx.model.isolationForestAnomaly =
        tml.isolationForestAnomaly;

    fusionInput.mlx.model.oneClassSVMAnomaly =
        tml.oneClassSVMAnomaly;

    fusionInput.mlx.model.bothModelsAnomaly =
        tml.bothModelsAnomaly;

    fusionInput.mlx.model.eitherModelAnomaly =
        tml.eitherModelAnomaly;

    fusionInput.mlx.model.isolationForestScore =
        tml.isolationForestDecision;

    fusionInput.mlx.model.oneClassSVMScore =
        tml.oneClassSVMDecision;

    fusionInput.mlx.model.confidence =
        tml.valid
            ? 1.0f
            : 0.0f;

    fusionInput.fsr.health =
        mapSensorHealth(
            fsrInitialized,
            f.connected,
            f.valid,
            f.status
            ==
            FSRStatus::CALIBRATING
        );

    fusionInput.fsr.reading =
        f;

    // Step 5.5 FSR model evidence. The model operates on
    // scale-invariant 9-sensor pressure-share features.
    fusionInput.fsr.model.available =
        fml.modelAvailable;

    fusionInput.fsr.model.valid =
        fml.valid;

    fusionInput.fsr.model.isolationForestAnomaly =
        fml.isolationForestAnomaly;

    fusionInput.fsr.model.oneClassSVMAnomaly =
        fml.oneClassSVMAnomaly;

    fusionInput.fsr.model.bothModelsAnomaly =
        fml.bothModelsAnomaly;

    fusionInput.fsr.model.eitherModelAnomaly =
        fml.eitherModelAnomaly;

    fusionInput.fsr.model.isolationForestScore =
        fml.isolationForestDecision;

    fusionInput.fsr.model.oneClassSVMScore =
        fml.oneClassSVMDecision;

    fusionInput.fsr.model.confidence =
        fml.valid
            ? 1.0f
            : 0.0f;

    fusionInput.mpu.health =
        mapSensorHealth(
            mpuInitialized,
            m.connected,
            m.valid,
            false
        );

    fusionInput.mpu.reading =
        m;

    // Step 5.6 MPU model evidence is road/vehicle-motion
    // artifact context. Fusion must never treat it as an
    // independent occupant-health anomaly vote.
    fusionInput.mpu.model.available =
        mml.modelAvailable;

    fusionInput.mpu.model.valid =
        mml.valid;

    fusionInput.mpu.model.isolationForestAnomaly =
        mml.isolationForestAnomaly;

    fusionInput.mpu.model.oneClassSVMAnomaly =
        mml.oneClassSVMAnomaly;

    fusionInput.mpu.model.bothModelsAnomaly =
        mml.bothModelsAnomaly;

    fusionInput.mpu.model.eitherModelAnomaly =
        mml.eitherModelAnomaly;

    fusionInput.mpu.model.isolationForestScore =
        mml.isolationForestDecision;

    fusionInput.mpu.model.oneClassSVMScore =
        mml.oneClassSVMDecision;

    fusionInput.mpu.model.confidence =
        mml.valid
            ? 1.0f
            : 0.0f;

    // Step 5.7.2 remote Piezo evidence. The separate Piezo ESP32
    // performs its own 25 Hz preprocessing + IF/OCSVM inference.
    // Main Hub receives only model/signal evidence over UART.
    fusionInput.piezo =
        piezoComm.getFusionEvidence();

    fusion.update(
        fusionInput
    );


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
        MAIN_PRINT_INTERVAL_MS
    )
    {
        return;
    }

    lastPrintTime =
        now;


    const FusionReading &fusionReading =
        fusion.getReading();

    const PiezoRemoteStatus &piezoStatus =
        piezoComm.getStatus();

    const PiezoFusionEvidence &piezoEvidence =
        fusionInput.piezo;

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
        "Init           : "
    );

    Serial.println(
        readyText(
            c1001Initialized
        )
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


    Serial.println();
    Serial.println(
        "C1001 ML:"
    );

    Serial.print(
        "  Status       : "
    );

    Serial.println(
        c1001ML.getStatusText()
    );

    Serial.print(
        "  Window       : "
    );

    Serial.print(
        cml.windowSamplesCollected
    );

    Serial.print(
        " / "
    );

    Serial.println(
        cml.windowSamplesRequired
    );

    Serial.print(
        "  Next infer   : "
    );

    Serial.print(
        cml.samplesUntilNextInference
    );

    Serial.println(
        " sample(s)"
    );

    Serial.print(
        "  Windows      : "
    );

    Serial.println(
        cml.windowsEvaluated
    );

    if (
        cml.valid
    )
    {
        Serial.print(
            "  IF decision  : "
        );

        Serial.print(
            cml.isolationForestDecision,
            6
        );

        Serial.println(
            cml.isolationForestAnomaly
                ? "  [ANOMALY]"
                : "  [NORMAL]"
        );

        Serial.print(
            "  SVM decision : "
        );

        Serial.print(
            cml.oneClassSVMDecision,
            6
        );

        Serial.println(
            cml.oneClassSVMAnomaly
                ? "  [ANOMALY]"
                : "  [NORMAL]"
        );

        Serial.print(
            "  Fusion vote  : "
        );

        if (
            cml.bothModelsAnomaly
        )
        {
            Serial.println(
                "STRONG ANOMALY"
            );
        }
        else if (
            cml.eitherModelAnomaly
        )
        {
            Serial.println(
                "WEAK ANOMALY"
            );
        }
        else
        {
            Serial.println(
                "NORMAL"
            );
        }
    }
    else
    {
        Serial.println(
            "  Model result : not ready"
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
        "Init           : "
    );

    Serial.println(
        readyText(
            mlxInitialized
        )
    );

    Serial.print(
        "Status         : "
    );

    Serial.println(
        mlx.getStatusText()
    );

    if (
        mlxInitialized
        &&
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
            "Temperature    : unavailable"
        );
    }


    Serial.println();

    Serial.println(
        "MLX WESAD ML (DIAGNOSTIC ONLY - NOT FUSED):"
    );

    Serial.print(
        "  Status       : "
    );

    Serial.println(
        mlxML.getStatusText()
    );

    Serial.print(
        "  Target gate  : "
    );

    Serial.println(
        tml.warmTargetQualified
            ? "QUALIFIED"
            : "NOT QUALIFIED"
    );

    if (
        isfinite(
            tml.targetDeltaC
        )
    )
    {
        Serial.print(
            "  Gate delta   : "
        );

        Serial.print(
            tml.targetDeltaC,
            2
        );

        Serial.println(
            " C (provisional min +2.00 C)"
        );
    }

    Serial.print(
        "  Window       : "
    );

    Serial.print(
        tml.windowSamplesCollected
    );

    Serial.print(
        " / "
    );

    Serial.println(
        tml.windowSamplesRequired
    );

    Serial.print(
        "  Next infer   : "
    );

    Serial.print(
        tml.samplesUntilNextInference
    );

    Serial.println(
        " sample(s)"
    );

    Serial.print(
        "  Windows      : "
    );

    Serial.println(
        tml.windowsEvaluated
    );

    if (
        tml.lastFiniteSampleCount
        >
        0
    )
    {
        Serial.print(
            "  Finite data  : "
        );

        Serial.print(
            tml.lastFiniteSampleCount
        );

        Serial.print(
            " / "
        );

        Serial.println(
            MLX_ML_WINDOW_SAMPLES
        );
    }

    if (
        tml.valid
    )
    {
        Serial.print(
            "  IF decision  : "
        );

        Serial.print(
            tml.isolationForestDecision,
            6
        );

        Serial.println(
            tml.isolationForestAnomaly
                ? "  [ANOMALY]"
                : "  [NORMAL]"
        );

        Serial.print(
            "  SVM decision : "
        );

        Serial.print(
            tml.oneClassSVMDecision,
            6
        );

        Serial.println(
            tml.oneClassSVMAnomaly
                ? "  [ANOMALY]"
                : "  [NORMAL]"
        );

        Serial.print(
            "  Diagnostic   : "
        );

        if (
            tml.bothModelsAnomaly
        )
        {
            Serial.println(
                "STRONG ANOMALY (NOT FUSED)"
            );
        }
        else if (
            tml.eitherModelAnomaly
        )
        {
            Serial.println(
                "WEAK ANOMALY (NOT FUSED)"
            );
        }
        else
        {
            Serial.println(
                "NORMAL (NOT FUSED)"
            );
        }
    }
    else
    {
        Serial.println(
            "  Model result : not ready"
        );
    }


    Serial.println();
    Serial.println(
        "MLX FUSION CONTEXT:"
    );

    Serial.print(
        "  Status       : "
    );
    Serial.println(
        mlxContext.getStatusText()
    );

    Serial.print(
        "  Thermal gate : "
    );
    Serial.println(
        tx.thermalContrastQualified
            ? "QUALIFIED"
            : "NOT QUALIFIED"
    );

    if (
        isfinite(
            tx.thermalContrastC
        )
    )
    {
        Serial.print(
            "  Object-Ta    : "
        );
        Serial.print(
            tx.thermalContrastC,
            2
        );
        Serial.println(
            " C (quality context only; NOT body temperature)"
        );
    }

    Serial.print(
        "  Baseline     : "
    );

    if (
        tx.baselineReady
        &&
        isfinite(
            tx.baselineObjectC
        )
    )
    {
        Serial.print(
            tx.baselineObjectC,
            2
        );
        Serial.println(
            " C"
        );
    }
    else
    {
        Serial.print(
            tx.baselineSamplesCollected
        );
        Serial.print(
            " / "
        );
        Serial.print(
            tx.baselineSamplesRequired
        );
        Serial.println(
            " filtered samples"
        );
    }

    if (
        tx.baselineReady
        &&
        isfinite(
            tx.deviationFromBaselineC
        )
    )
    {
        Serial.print(
            "  Deviation    : "
        );
        Serial.print(
            tx.deviationFromBaselineC,
            2
        );
        Serial.println(
            " C"
        );

        Serial.println(
            "  FDA context  : +/-1.85 C broad p99 repeated-round reference (non-medical)"
        );
    }

    Serial.print(
        "  Fusion role  : "
    );

    if (
        tx.valid
        &&
        tx.contextChange
    )
    {
        Serial.println(
            "SUPPORTING CONTEXT CHANGE ONLY - cannot create warning alone"
        );
    }
    else if (
        tx.valid
    )
    {
        Serial.println(
            "STABLE TEMPERATURE CONTEXT"
        );
    }
    else
    {
        Serial.println(
            "NOT YET VALID FOR FUSION"
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
        "Init           : "
    );

    Serial.println(
        readyText(
            fsrInitialized
        )
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
        fsrInitialized
        &&
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
    else
    {
        Serial.println(
            "Pressure data  : unavailable"
        );
    }



    // ========================================================
    // FSR EMBEDDED ML - STEP 5.5
    // ========================================================

    Serial.println();
    Serial.println(
        "FSR ML:"
    );

    Serial.print(
        "  Status       : "
    );

    Serial.println(
        fsrML.getStatusText()
    );

    Serial.print(
        "  Window       : "
    );

    Serial.print(
        fml.windowSamplesCollected
    );

    Serial.print(
        " / "
    );

    Serial.println(
        fml.windowSamplesRequired
    );

    Serial.print(
        "  Next infer   : "
    );

    Serial.print(
        fml.samplesUntilNextInference
    );

    Serial.println(
        " frame(s)"
    );

    Serial.print(
        "  Windows      : "
    );

    Serial.println(
        fml.windowsEvaluated
    );

    Serial.println(
        "  Input        : 9-sensor pressure shares (absolute scale removed)"
    );

    if (
        fml.valid
    )
    {
        Serial.print(
            "  IF decision  : "
        );

        Serial.print(
            fml.isolationForestDecision,
            6
        );

        Serial.print(
            " -> "
        );

        Serial.println(
            fml.isolationForestAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.print(
            "  SVM decision : "
        );

        Serial.print(
            fml.oneClassSVMDecision,
            6
        );

        Serial.print(
            " -> "
        );

        Serial.println(
            fml.oneClassSVMAnomaly
                ? "ANOMALY"
                : "NORMAL"
        );

        Serial.print(
            "  Fused result : "
        );

        if (
            fml.bothModelsAnomaly
        )
        {
            Serial.println(
                "STRONG ANOMALY"
            );
        }
        else if (
            fml.eitherModelAnomaly
        )
        {
            Serial.println(
                "WEAK ANOMALY"
            );
        }
        else
        {
            Serial.println(
                "NORMAL"
            );
        }
    }
    else
    {
        Serial.println(
            "  Model result : not ready"
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
        "Init           : "
    );

    Serial.println(
        readyText(
            mpuInitialized
        )
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
        mpuInitialized
        &&
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
    else
    {
        Serial.println(
            "Motion data    : unavailable"
        );
    }


    Serial.println();
    Serial.println(
        "MPU ML (ROAD/MOTION CONTEXT):"
    );

    Serial.print(
        "  Status       : "
    );
    Serial.println(
        mpuML.getStatusText()
    );

    Serial.print(
        "  Baseline     : "
    );

    if (
        mml.stationaryBaselineReady
    )
    {
        Serial.println(
            "READY"
        );
    }
    else
    {
        Serial.print(
            mml.baselineSamplesCollected
        );
        Serial.print(
            " / "
        );
        Serial.println(
            mml.baselineSamplesRequired
        );
    }

    Serial.print(
        "  Window       : "
    );
    Serial.print(
        mml.windowSamplesCollected
    );
    Serial.print(
        " / "
    );
    Serial.println(
        mml.windowSamplesRequired
    );

    Serial.print(
        "  Next infer   : "
    );
    Serial.print(
        mml.samplesUntilNextInference
    );
    Serial.println(
        " sample(s)"
    );

    Serial.print(
        "  Windows      : "
    );
    Serial.println(
        mml.windowsEvaluated
    );

    if (
        mml.valid
    )
    {
        Serial.print(
            "  IF decision  : "
        );
        Serial.println(
            mml.isolationForestDecision,
            6
        );

        Serial.print(
            "  SVM decision : "
        );
        Serial.println(
            mml.oneClassSVMDecision,
            6
        );

        Serial.print(
            "  Context      : "
        );

        if (
            mml.bothModelsAnomaly
        )
        {
            Serial.println(
                "STRONG ROAD/MOTION ANOMALY"
            );
        }
        else if (
            mml.eitherModelAnomaly
        )
        {
            Serial.println(
                "WEAK ROAD/MOTION ANOMALY"
            );
        }
        else
        {
            Serial.println(
                "NORMAL ROAD MOTION"
            );
        }

        Serial.println(
            "  Fusion role  : ARTIFACT CONTEXT ONLY"
        );
    }
    else
    {
        Serial.println(
            "  Model result : not ready"
        );
    }


    // ========================================================
    // PIEZO REMOTE - STEP 5.7.2
    // ========================================================

    Serial.println();
    Serial.println(
        "--------------- PIEZO --------------------"
    );

    Serial.print(
        "Link init      : "
    );
    Serial.println(
        readyText(
            piezoCommInitialized
        )
    );

    Serial.print(
        "Connected      : "
    );
    Serial.println(
        piezoStatus.connected
            ? "YES"
            : "NO / STALE"
    );

    Serial.print(
        "Packets RX     : "
    );
    Serial.println(
        piezoStatus.packetsReceived
    );

    Serial.print(
        "Bad packets    : "
    );
    Serial.println(
        piezoStatus.badPackets
    );

    if (piezoStatus.connected)
    {
        Serial.print(
            "Packet age     : "
        );
        Serial.print(
            piezoStatus.packetAgeMillis
        );
        Serial.println(
            " ms"
        );

        Serial.print(
            "Remote Fs      : "
        );
        Serial.print(
            piezoStatus.remoteSamplingRateHz,
            2
        );
        Serial.println(
            " Hz"
        );

        Serial.print(
            "Feature windows: "
        );
        Serial.println(
            piezoStatus.remoteFeatureWindowCount
        );

        Serial.print(
            "Signal quality : "
        );
        Serial.println(
            piezoEvidence.signalQualityValid
                ? "VALID"
                : "NOT READY / REJECTED"
        );

        Serial.print(
            "Peak RR        : "
        );
        if (
            isfinite(
                piezoEvidence.peakRespirationBPM
            )
        )
        {
            Serial.print(
                piezoEvidence.peakRespirationBPM,
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
            "Spectral RR    : "
        );
        if (
            isfinite(
                piezoEvidence.spectralRespirationBPM
            )
        )
        {
            Serial.print(
                piezoEvidence.spectralRespirationBPM,
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
            "Aux no-breath  : "
        );
        Serial.println(
            piezoEvidence.noBreathTimerExceeded
                ? "TIMER EXCEEDED (context only)"
                : "NO"
        );

        Serial.print(
            "Model          : "
        );

        if (!piezoEvidence.model.valid)
        {
            Serial.println(
                "not ready"
            );
        }
        else
        {
            if (
                piezoEvidence.model.bothModelsAnomaly
            )
            {
                Serial.println(
                    "STRONG RESPIRATION-PATTERN ANOMALY"
                );
            }
            else if (
                piezoEvidence.model.eitherModelAnomaly
            )
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

            Serial.print(
                "  IF decision  : "
            );
            Serial.println(
                piezoEvidence.model.isolationForestScore,
                6
            );

            Serial.print(
                "  SVM decision : "
            );
            Serial.println(
                piezoEvidence.model.oneClassSVMScore,
                6
            );
        }

        Serial.println(
            "Fusion role    : respiration corroboration; never standalone emergency"
        );
    }


    // ========================================================
    // FUSION
    // ========================================================

    Serial.println();
    Serial.println(
        "------------- FUSION ------------------"
    );

    Serial.print(
        "Valid          : "
    );
    Serial.println(
        fusionReading.valid
            ? "YES"
            : "NO"
    );

    Serial.print(
        "Occupancy      : "
    );
    Serial.println(
        fusion.getOccupancyText(
            fusionReading.occupancy
        )
    );

    Serial.print(
        "Motion         : "
    );
    Serial.println(
        fusion.getMotionText(
            fusionReading.motion
        )
    );

    Serial.print(
        "Vitals         : "
    );
    Serial.println(
        fusion.getVitalsText(
            fusionReading.vitals
        )
    );

    Serial.print(
        "Pressure       : "
    );
    Serial.println(
        fusion.getPressureText(
            fusionReading.pressure
        )
    );

    Serial.print(
        "Temperature    : "
    );
    Serial.println(
        fusion.getTemperatureText(
            fusionReading.temperature
        )
    );

    Serial.print(
        "Respiration    : "
    );
    Serial.println(
        fusion.getRespirationText(
            fusionReading.respiration
        )
    );

    Serial.print(
        "Level          : "
    );
    Serial.println(
        fusion.getLevelText(
            fusionReading.level
        )
    );

    Serial.print(
        "Confidence     : "
    );
    Serial.print(
        fusionReading.confidence,
        2
    );
    Serial.println();


    // ========================================================
    // UPCOMING SYSTEM LAYERS
    // ========================================================

    Serial.println();
    Serial.println(
        "------------------------------------------"
    );

    Serial.println(
        "Piezo          : separate ESP32; UART evidence link ACTIVE"
    );

    Serial.println(
        "ML inference   : C1001 + FSR + MPU + Piezo ACTIVE; MLX WESAD diagnostic-only"
    );

    Serial.println(
        "Fusion         : active; C1001 + MLX context + FSR + MPU motion context + Piezo corroboration"
    );

    Serial.println(
        "Camera         : separate ESP32-CAM"
    );

    Serial.println(
        "=========================================="
    );
}
