#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

#include "C1001.h"
#include "C1001Comm.h"
#include "MLX.h"
#include "MLXML.h"
#include "MLXContext.h"
#include "FSR.h"
#include "FSRML.h"
#include "MPU.h"
#include "MPUML.h"
#include "Fusion.h"
#include "PiezoComm.h"
#include "CameraComm.h"
#include "SafeSeatAccessPoint.h"
#include "NetworkConfig.h"
#include "SafeSeatTelemetry.h"
#include "SafeSeatApi.h"


// ============================================================
// SENSOR OBJECTS
// ============================================================

C1001Comm c1001Comm;
MLXSensor mlx;
MLXML mlxML;
MLXContext mlxContext;
FSRSensor fsr;
FSRML fsrML;
MPUSensor mpu;
MPUML mpuML;
FusionEngine fusion;
PiezoComm piezoComm;
CameraComm cameraComm;
SafeSeatAccessPoint safeSeatAccessPoint;
SafeSeatTelemetry safeSeatTelemetry;
SafeSeatApi safeSeatApi;


// ============================================================
// INITIALIZATION STATE
//
// These flags describe what actually happened during begin().
// We do NOT print "acquisition active" for a module that failed
// initialization.
// ============================================================

bool c1001CommInitialized = false;
bool mlxInitialized = false;
bool fsrInitialized = false;
bool mpuInitialized = false;
bool piezoCommInitialized = false;
bool cameraCommInitialized = false;
bool safeSeatAccessPointInitialized = false;
bool safeSeatApiInitialized = false;


// ============================================================
// REMOTE C1001
//
// Step 5.8 moves M1A (C1001 + C1001 ML) to its own ESP32.
// The Main Hub receives its current evidence over ESP-NOW.
// ============================================================


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
        "C1001Link: "
    );
    Serial.println(
        readyText(
            c1001CommInitialized
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

    Serial.print(
        "CameraLink: "
    );
    Serial.println(
        readyText(
            cameraCommInitialized
        )
    );

    Serial.print(
        "SafeSeatAP: "
    );
    Serial.println(
        readyText(
            safeSeatAccessPointInitialized
        )
    );

    Serial.print(
        "Local API : "
    );
    Serial.println(
        readyText(
            safeSeatApiInitialized
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
        " Step 5.9.8.2 - Deterministic Piezo + API + SoftAP + ESP-NOW"
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
    // SAFESEAT LOCAL WI-FI ACCESS POINT - STEP 5.9.6
    //
    // Main Hub is the only device that creates the SafeSeat AP.
    // The ESP32-S3 camera and phone join this network. C1001 and
    // Piezo continue to use ESP-NOW through the station interface.
    // Step 5.9.8 adds a read-only local telemetry/API layer on
    // top of this existing network.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting SafeSeat local Wi-Fi network..."
    );

    safeSeatAccessPointInitialized =
        safeSeatAccessPoint.begin();

    Serial.println(
        safeSeatAccessPointInitialized
            ? "[MAIN] SafeSeat Wi-Fi AP ready."
            : "[MAIN] WARNING: SafeSeat Wi-Fi AP failed."
    );


    // ========================================================
    // REMOTE C1001 LINK - STEP 5.8
    //
    // M1A is now a dashboard module containing its own C1001,
    // dedicated ESP32, filtering, and trained IF + OCSVM model.
    // Main Hub receives only the current sensor/model evidence.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting remote C1001 evidence link..."
    );

    c1001CommInitialized =
        c1001Comm.begin();

    Serial.println(
        c1001CommInitialized
            ? "[MAIN] ESP-NOW ready; waiting for C1001 node."
            : "[MAIN] WARNING: C1001 ESP-NOW link failed."
    );


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
    // PIEZO REMOTE ESP-NOW - STEP 5.9.8.2
    //
    // Wireless deterministic respiration support from the separate
    // seatbelt ESP32. No Piezo ML model is deployed. No Piezo/Main
    // signal wire or common-ground link is required.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting Piezo evidence link..."
    );

    piezoCommInitialized =
        piezoComm.begin();

    Serial.println(
        piezoCommInitialized
            ? "[MAIN] SafeSeat ESP-NOW ready; waiting for Piezo evidence."
            : "[MAIN] WARNING: ESP-NOW initialization failed."
    );


    // ========================================================
    // ESP32-S3 CAMERA VERIFICATION LINK - STEP 5.9.5
    //
    // The camera is a separate ESP32-S3 WROOM camera node. It remains idle
    // until Fusion creates a persistent strong candidate.
    // ========================================================

    Serial.println();
    Serial.println(
        "[MAIN] Starting ESP32-S3 camera verification link..."
    );

    cameraCommInitialized =
        cameraComm.begin();

    Serial.println(
        cameraCommInitialized
            ? "[MAIN] Camera ESP-NOW transport ready; waiting for camera heartbeat."
            : "[MAIN] WARNING: Camera ESP-NOW link failed."
    );


    fusion.begin();


    // ========================================================
    // LOCAL TELEMETRY / API - STEP 5.9.8
    //
    // This is intentionally read-only and backend-agnostic.
    // It exposes the latest Main Hub/Fusion state to a phone or
    // future frontend connected to the SafeSeat Wi-Fi network.
    // No endpoint can trigger/cancel camera verification or alter
    // the Fusion decision.
    // ========================================================

    if (safeSeatAccessPointInitialized)
    {
        safeSeatApiInitialized =
            safeSeatApi.begin(
                &safeSeatTelemetry
            );
    }
    else
    {
        Serial.println(
            "[API] WARNING: SafeSeat AP unavailable; local API not started."
        );
    }

    printInitializationSummary();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    if (safeSeatAccessPointInitialized)
    {
        safeSeatAccessPoint.update();
    }

    // ========================================================
    // UPDATE ONLY MODULES THAT ACTUALLY INITIALIZED
    //
    // Failed/disconnected sensors are not treated as normal
    // data. Their reading structures remain available for the
    // dashboard and, later, Fusion validity gating.
    // ========================================================

    if (
        c1001CommInitialized
    )
    {
        c1001Comm.update();
    }

    if (
        piezoCommInitialized
    )
    {
        piezoComm.update();
    }

    if (
        cameraCommInitialized
    )
    {
        cameraComm.update();
    }

    if (
        mpuInitialized
    )
    {
        mpu.update();
    }

    if (
        mlxInitialized
    )
    {
        mlx.update();
    }

    const C1001RemoteStatus &c1001Remote =
        c1001Comm.getStatus();

    const C1001Reading &c =
        c1001Comm.getReading();

    const ModelEvidence &cml =
        c1001Comm.getModelEvidence();

    // Only fresh remote C1001 presence is used as independent
    // occupancy context. FSR still retains its own calibrated
    // occupied/back-contact gates if the remote node is absent.
    bool occupantPresent =
        c1001Remote.connected
        &&
        c.present;

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


    // C1001 acquisition + ML now run on the remote M1A node.
    // c / cml above are fresh ESP-NOW evidence snapshots.

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
        occupantPresent
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

    // Remote C1001 node already performed acquisition, filtering,
    // warm-up logic, feature extraction, IF, and OCSVM inference.
    // Fusion receives the exact same C1001FusionInput contract.
    fusionInput.c1001 =
        c1001Comm.getFusionInput();

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

    // Step 5.9.8.2 remote Piezo support. The separate Piezo ESP32
    // performs 25 Hz deterministic signal processing and breath-event
    // tracking. No IF/OCSVM is deployed on the Piezo node.
    fusionInput.piezo =
        piezoComm.getFusionEvidence();

    // Step 5.9.4 ESP32-CAM evidence. Camera output is verification
    // only: UPRIGHT can reject one persistent strong candidate;
    // any leaning class can confirm it. Transaction IDs prevent
    // stale camera results from affecting later candidates.
    fusionInput.camera =
        cameraComm.getFusionEvidence();

    fusion.update(
        fusionInput
    );

    const FusionReading &fusionReading =
        fusion.getReading();

    // Trigger/cancel the camera transaction AFTER Fusion computes
    // whether verification is actually required.
    if (cameraCommInitialized)
    {
        cameraComm.serviceVerificationRequest(
            fusionReading.triggerCamera
        );
    }


    // ========================================================
    // FRONTEND-READY TELEMETRY SNAPSHOT - STEP 5.9.8
    //
    // Capture AFTER Fusion and camera-request servicing so the
    // API sees the latest authoritative system state and current
    // camera transaction status. This copy is read-only.
    // ========================================================

    safeSeatTelemetry.capture(
        fusionInput,
        fusionReading,
        c1001Comm.getStatus(),
        piezoComm.getStatus(),
        cameraComm.getStatus(),
        safeSeatAccessPoint.getStatus()
    );

    // IMPORTANT: service HTTP clients before the Serial dashboard
    // rate-limit return below. The API therefore remains responsive
    // even when no dashboard print is due this loop iteration.
    if (safeSeatApiInitialized)
    {
        safeSeatApi.update();
    }


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


    const PiezoRemoteStatus &piezoStatus =
        piezoComm.getStatus();

    const PiezoFusionEvidence &piezoEvidence =
        fusionInput.piezo;

    const CameraRemoteStatus &cameraStatus =
        cameraComm.getStatus();

    const CameraFusionEvidence &cameraEvidence =
        fusionInput.camera;

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
    // C1001 REMOTE NODE
    // ========================================================

    Serial.println();
    Serial.println(
        "----------- C1001 REMOTE NODE ------------"
    );

    Serial.print("Link init      : ");
    Serial.println(readyText(c1001CommInitialized));

    Serial.print("Connected      : ");
    Serial.println(c1001Remote.connected ? "YES" : "NO");

    Serial.print("Packets RX     : ");
    Serial.println(c1001Remote.packetsReceived);

    Serial.print("Bad packets    : ");
    Serial.println(c1001Remote.badPackets);

    Serial.print("Packet age     : ");
    Serial.print(c1001Remote.packetAgeMillis);
    Serial.println(" ms");

    Serial.print("Wi-Fi channel  : ");
    Serial.println(c1001Remote.linkChannel);

    if (c1001Remote.connected)
    {
        Serial.print("Node MAC       : ");
        for (uint8_t i = 0; i < 6; i++)
        {
            if (i > 0) Serial.print(':');
            if (c1001Remote.sourceMac[i] < 16) Serial.print('0');
            Serial.print(c1001Remote.sourceMac[i], HEX);
        }
        Serial.println();

        Serial.print("Sensor status  : ");
        Serial.println(c1001StatusText(c.status));

        Serial.print("Presence       : ");
        Serial.println(c.present ? "YES" : "NO");

        Serial.print("MoveRange      : ");
        Serial.println(c.moveRange);

        Serial.print("Raw RR / HR    : ");
        Serial.print(c.rawRespiration);
        Serial.print(" / ");
        Serial.println(c.rawHeartRate);

        if (c.status == C1001Status::WARMING_UP)
        {
            Serial.print("Warm-up left   : ");
            Serial.print(c.warmupRemainingSeconds);
            Serial.println(" s");
        }

        if (c.trustedVitalsAvailable)
        {
            Serial.print("Filtered RR    : ");
            Serial.print(c.filteredRespiration, 1);
            Serial.println(" BPM");

            Serial.print("Filtered HR    : ");
            Serial.print(c.filteredHeartRate, 1);
            Serial.println(" BPM");
        }
        else
        {
            Serial.println("Filtered vitals: not ready");
        }

        Serial.println();
        Serial.println("C1001 ML (REMOTE):");
        Serial.print("  Window       : ");
        Serial.print(c1001Remote.remoteWindowSamplesCollected);
        Serial.print(" / ");
        Serial.print(c1001Remote.remoteWindowSamplesRequired);
        Serial.println();
        Serial.print("  Next infer   : ");
        Serial.print(c1001Remote.remoteSamplesUntilNextInference);
        Serial.println(" sample(s)");
        Serial.print("  Windows      : ");
        Serial.println(c1001Remote.remoteWindowsEvaluated);

        if (cml.valid)
        {
            Serial.print("  IF decision  : ");
            Serial.print(cml.isolationForestScore, 6);
            Serial.println(cml.isolationForestAnomaly ? " [ANOMALY]" : " [NORMAL]");

            Serial.print("  SVM decision : ");
            Serial.print(cml.oneClassSVMScore, 6);
            Serial.println(cml.oneClassSVMAnomaly ? " [ANOMALY]" : " [NORMAL]");

            Serial.print("  Fusion vote  : ");
            if (cml.bothModelsAnomaly) Serial.println("STRONG ANOMALY");
            else if (cml.eitherModelAnomaly) Serial.println("WEAK ANOMALY");
            else Serial.println("NORMAL");
        }
        else
        {
            Serial.println("  Model result : not ready");
        }
    }
    else
    {
        Serial.println("Remote C1001   : unavailable / waiting for node");
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
    // SAFESEAT LOCAL NETWORK - STEP 5.9.6
    // ========================================================

    Serial.println();
    Serial.println(
        "------------- NETWORK -----------------"
    );

    const SafeSeatAccessPointStatus &apStatus =
        safeSeatAccessPoint.getStatus();

    Serial.print("SafeSeat AP    : ");
    Serial.println(
        apStatus.running
            ? "ONLINE"
            : "OFFLINE"
    );

    Serial.print("SSID           : ");
    Serial.println(SAFESEAT_AP_SSID);

    Serial.print("IP             : ");
    Serial.println(apStatus.ipAddress);

    Serial.print("Channel        : ");
    Serial.println(apStatus.channel);

    Serial.print("Wi-Fi clients  : ");
    Serial.println(apStatus.connectedClients);

    Serial.println(
        "Role           : local phone/camera network; no Internet required"
    );


    // ========================================================
    // PIEZO REMOTE - STEP 5.9.8.2
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

    const SafeSeatNowStatus &wirelessStatus =
        piezoComm.getWirelessStatus();

    Serial.print(
        "Transport      : "
    );
    Serial.println(
        "ESP-NOW"
    );

    Serial.print(
        "Wi-Fi channel  : "
    );
    Serial.println(
        wirelessStatus.channel
    );

    Serial.print(
        "Hub beacons TX : "
    );
    Serial.println(
        wirelessStatus.hubBeaconsSent
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

        Serial.printf(
            "Piezo MAC      : %02X:%02X:%02X:%02X:%02X:%02X\n",
            piezoStatus.sourceMac[0],
            piezoStatus.sourceMac[1],
            piezoStatus.sourceMac[2],
            piezoStatus.sourceMac[3],
            piezoStatus.sourceMac[4],
            piezoStatus.sourceMac[5]
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
            "Breath events  : "
        );
        Serial.println(
            piezoStatus.remoteBreathCount
        );

        Serial.print(
            "Signal usable  : "
        );
        Serial.println(
            piezoEvidence.signalUsable
                ? "YES"
                : "STARTING / INVALID"
        );

        Serial.print(
            "Tracking ready : "
        );
        Serial.println(
            piezoEvidence.breathTrackingReady
                ? "YES"
                : "NO"
        );

        Serial.print(
            "Respiration RR : "
        );
        if (
            isfinite(
                piezoEvidence.respirationBPM
            )
        )
        {
            Serial.print(
                piezoEvidence.respirationBPM,
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
            "Resp waveform  : "
        );
        Serial.println(
            piezoEvidence.respirationWave,
            2
        );

        Serial.print(
            "Last event age : "
        );
        Serial.print(
            piezoEvidence.noBreathDurationMs
            /
            1000.0f,
            1
        );
        Serial.println(
            " sec"
        );

        Serial.print(
            "No-breath support: "
        );
        Serial.println(
            piezoEvidence.noBreathTimerExceeded
                ? "ACTIVE (CORROBORATION ONLY)"
                : "NO"
        );

        Serial.println(
            "Model          : NOT DEPLOYED"
        );

        Serial.println(
            "Fusion role    : deterministic respiration corroboration; never standalone emergency"
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
    // ESP32-CAM VERIFICATION - STEP 5.9.4
    // ========================================================

    Serial.println();
    Serial.println(
        "----------- ESP32-CAM VERIFICATION --------"
    );

    Serial.print("Transport      : ");
    Serial.println(cameraCommInitialized ? "ESP-NOW" : "UNAVAILABLE");
    Serial.print("Connected      : ");
    Serial.println(cameraStatus.connected ? "YES" : "NO / WAITING");
    Serial.print("Model ready    : ");
    Serial.println(cameraStatus.modelReady ? "YES" : "NO");
    Serial.print("Camera selftest: ");
    Serial.println(cameraStatus.cameraReady ? "PASS" : "NOT READY");
    Serial.print("PSRAM          : ");
    Serial.println(cameraStatus.psramReady ? "READY" : "NOT READY");
    Serial.print("Remote busy    : ");
    Serial.println(cameraStatus.busy ? "YES - VERIFYING" : "NO / IDLE");
    Serial.print("Status RX      : ");
    Serial.println(cameraStatus.statusPacketsReceived);
    Serial.print("Results RX     : ");
    Serial.println(cameraStatus.resultPacketsReceived);
    Serial.print("Request active : ");
    Serial.println(cameraStatus.requestActive ? "YES" : "NO");

    if (cameraStatus.requestActive)
    {
        Serial.print("Request ID     : ");
        Serial.println(cameraStatus.activeRequestId);
        Serial.print("Request age    : ");
        Serial.print(cameraStatus.requestAgeMillis);
        Serial.println(" ms");
    }

    if (cameraStatus.lastResultRequestId != 0)
    {
        Serial.print("Last posture   : ");
        Serial.println(cameraPostureText(cameraStatus.lastPosture));
        Serial.print("Confidence     : ");
        Serial.print(cameraStatus.lastConfidence * 100.0f, 1);
        Serial.println("%");
        Serial.print("Valid frames   : ");
        Serial.println(cameraStatus.lastValidFrames);
        Serial.print("Verify time    : ");
        Serial.print(cameraStatus.lastRemoteInferenceMillis);
        Serial.println(" ms");
        Serial.print("Fusion result  : ");
        if (cameraEvidence.resultValid)
        {
            Serial.println(
                cameraEvidence.postureNormal
                    ? "UPRIGHT / NORMAL VERIFICATION"
                    : "LEANING / ABNORMAL VERIFICATION"
            );
        }
        else
        {
            Serial.println("no fresh accepted result");
        }
    }
    else
    {
        Serial.println("Last result    : none yet");
    }


    // ========================================================
    // UPCOMING SYSTEM LAYERS
    // ========================================================

    Serial.println();
    Serial.println(
        "------------------------------------------"
    );

    Serial.println(
        "Piezo          : separate ESP32; ESP-NOW evidence link ACTIVE"
    );

    Serial.println(
        "ML inference   : C1001 REMOTE + FSR + MPU + Piezo ACTIVE; MLX WESAD diagnostic-only"
    );

    Serial.println(
        "Fusion         : active; C1001 + MLX context + FSR + MPU context + Piezo + camera verification"
    );

    Serial.println(
        "Camera         : ESP32-CAM INT8 verifier + ESP-NOW trigger/result ACTIVE"
    );

    Serial.println(
        "=========================================="
    );
}
