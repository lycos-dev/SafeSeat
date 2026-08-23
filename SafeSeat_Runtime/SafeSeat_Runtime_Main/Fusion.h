#pragma once

#include <Arduino.h>

#include "C1001.h"
#include "MLX.h"
#include "MLXContext.h"
#include "FSR.h"
#include "MPU.h"


// ============================================================
// SAFESEAT SENSOR FUSION CORE - STEP 5.1
//
// This header defines the common data contracts used by the
// fusion layer.
//
// IMPORTANT:
// - No Isolation Forest / OCSVM implementation lives here.
// - Sensor-specific ML remains inside each sensor pipeline.
// - Fusion receives only the RESULTS of those models.
// - C1001 arrives over ESP-NOW.
// - Camera verification is received from the ESP32-S3 camera over ESP-NOW.
// ============================================================


// ============================================================
// SENSOR HEALTH
//
// Fusion must distinguish "normal" from "not available".
// A disconnected or warming-up sensor MUST NOT count as
// evidence that the occupant is normal.
// ============================================================

enum class FusionSensorHealth
{
    UNAVAILABLE,
    INITIALIZING,
    WARMING_UP,
    VALID,
    DEGRADED,
    INVALID
};


// ============================================================
// MODEL EVIDENCE
//
// Generic container for any SafeSeat anomaly model.
//
// Each sensor-specific runtime can later fill this structure
// after running:
// - scaler
// - Isolation Forest
// - One-Class SVM
//
// Fusion itself does not know how those models work.
// ============================================================

struct ModelEvidence
{
    // Has this model pipeline been integrated into runtime?
    bool available = false;

    // Was the current model window valid enough for inference?
    bool valid = false;

    // Individual model decisions.
    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    // Combined decisions.
    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;

    // Continuous decision values.
    //
    // Convention used by our anomaly runtimes:
    // positive / higher = more normal
    // negative / lower   = more anomalous
    //
    // Exact scale is model-specific and Fusion should not
    // compare scores from different sensor types directly.
    float isolationForestScore = NAN;
    float oneClassSVMScore = NAN;

    // Optional sensor-level confidence.
    //
    // 0.0 = no confidence
    // 1.0 = maximum confidence
    //
    // This will be populated only where the sensor runtime has
    // a meaningful confidence calculation.
    float confidence = 0.0f;
};


// ============================================================
// OCCUPANCY STATE
// ============================================================

enum class FusionOccupancyState
{
    UNKNOWN,
    EMPTY,
    OCCUPIED,
    CONFLICT
};


// ============================================================
// MOTION / VEHICLE CONTEXT
//
// MPU6050 is primarily contextual:
// road vibration / bumps / seat motion should be able to
// explain temporary pressure or physiological artifacts.
// ============================================================

enum class FusionMotionState
{
    UNKNOWN,
    STILL,
    LOW_MOTION,
    MODERATE_MOTION,
    HIGH_MOTION
};


// ============================================================
// VITALS STATE
// ============================================================

enum class FusionVitalsState
{
    UNKNOWN,
    NOT_READY,
    NORMAL,
    ANOMALOUS
};


// ============================================================
// PRESSURE / POSTURE STATE
// ============================================================

enum class FusionPressureState
{
    UNKNOWN,
    EMPTY,
    NORMAL,
    ASYMMETRIC,
    CONTACT_LOSS,
    ANOMALOUS
};


// ============================================================
// TEMPERATURE CONTEXT
//
// MLX90614 currently provides environmental/body-surface
// context.
//
// It is NOT treated as a standalone diagnosis.
// ============================================================

enum class FusionTemperatureState
{
    UNKNOWN,
    INVALID,
    NO_THERMAL_TARGET,
    BASELINE_BUILDING,
    STABLE,
    CONTEXT_CHANGE,
    ANOMALOUS
};


// ============================================================
// RESPIRATION STATE
//
// C1001 is the final deployed respiration source. This state is
// descriptive system context and is not a medical diagnosis.
// ============================================================

enum class FusionRespirationState
{
    UNKNOWN,
    NOT_AVAILABLE,
    NORMAL,
    IRREGULAR,
    NO_BREATH,
    ANOMALOUS
};


// ============================================================
// FINAL FUSION LEVEL
//
// These are system-severity states, NOT medical diagnoses.
//
// SAFE:
// no meaningful abnormal evidence.
//
// WATCH:
// incomplete data or weak isolated evidence.
//
// WARNING:
// multiple concerning signals or persistent anomaly.
//
// EMERGENCY:
// strong multi-sensor emergency candidate.
// Camera verification is requested from here.
// ============================================================

enum class FusionLevel
{
    SAFE,
    WATCH,
    WARNING,
    EMERGENCY
};



// ============================================================
// CAMERA VERIFICATION - STEP 5.9.4
//
// Fusion requests verification from the separate ESP32-S3 camera node.
// Main accepts only transaction-matched, fresh ESP-NOW results.
// ============================================================

struct CameraFusionEvidence
{
    // Communication / camera integration status.
    bool available = false;
    bool connected = false;

    // Has a recent verification result been received?
    bool resultValid = false;

    // Camera-side posture verification.
    bool postureNormal = false;
    bool postureAbnormal = false;

    // Model softmax/consensus confidence from the camera node.
    float confidence = 0.0f;

    // Transaction identity. resultId increments at the camera node
    // and lets Fusion process a normal result exactly once while
    // latching an abnormal result only for the current strong
    // emergency candidate.
    uint32_t requestId = 0;
    uint32_t resultId = 0;
    uint8_t postureClass = 255;

    unsigned long lastUpdateMillis = 0;
};


// ============================================================
// SENSOR-SPECIFIC FUSION INPUT
//
// Contains acquisition health + optional model result.
// ============================================================

struct C1001FusionInput
{
    FusionSensorHealth health =
        FusionSensorHealth::UNAVAILABLE;

    C1001Reading reading;

    ModelEvidence model;
};


struct MLXFusionInput
{
    FusionSensorHealth health =
        FusionSensorHealth::UNAVAILABLE;

    MLXReading reading;

    // Deployment-safe MLX context evidence.
    // Filtered OBJECT temperature is primary; MLX Ta and
    // Object-Ta remain context/quality only.
    MLXContextReading context;

    // 2026-08-23: actual-MLX90614 external-data model.
    // Personal/session-baseline-relative IF + OCSVM evidence is
    // now eligible for conservative Fusion use.
    ModelEvidence model;
};


struct FSRFusionInput
{
    FusionSensorHealth health =
        FusionSensorHealth::UNAVAILABLE;

    FSRReading reading;

    ModelEvidence model;
};


struct MPUFusionInput
{
    FusionSensorHealth health =
        FusionSensorHealth::UNAVAILABLE;

    MPUReading reading;

    ModelEvidence model;
};


// ============================================================
// COMPLETE FUSION INPUT
//
// This is the single object Fusion.cpp will evaluate.
//
// Local Main Hub sensors: MLX90614, FSR array, MPU6050.
// Remote ESP-NOW nodes: C1001 and ESP32-S3 camera.
// Camera is verification-only and never creates the underlying
// strong emergency candidate by itself.
// ============================================================

struct FusionInput
{
    C1001FusionInput c1001;
    MLXFusionInput mlx;
    FSRFusionInput fsr;
    MPUFusionInput mpu;

    CameraFusionEvidence camera;

    unsigned long timestampMillis = 0;
};


// ============================================================
// EVIDENCE SUMMARY
//
// Fusion.cpp will count independent supporting and conflicting
// pieces of evidence instead of blindly trusting one sensor.
// ============================================================

struct FusionEvidenceSummary
{
    uint8_t validSensorCount = 0;

    uint8_t unavailableSensorCount = 0;

    uint8_t anomalyEvidenceCount = 0;

    uint8_t strongAnomalyEvidenceCount = 0;

    uint8_t normalEvidenceCount = 0;

    // Non-anomaly contextual support. Step 5.4.5 uses this for
    // MLX session-baseline changes. It can strengthen confidence
    // around other sensor evidence but cannot create an anomaly
    // candidate by itself.
    uint8_t supportingContextCount = 0;

    // Context suggesting a possible artifact instead of a real
    // physiological/postural event.
    bool motionArtifactPossible = false;

    // Strong independent agreement.
    bool multiSensorAgreement = false;
};


// ============================================================
// FUSION OUTPUT
// ============================================================

struct FusionReading
{
    bool valid = false;


    // --------------------------------------------------------
    // Context classifications
    // --------------------------------------------------------

    FusionOccupancyState occupancy =
        FusionOccupancyState::UNKNOWN;

    FusionMotionState motion =
        FusionMotionState::UNKNOWN;

    FusionVitalsState vitals =
        FusionVitalsState::UNKNOWN;

    FusionPressureState pressure =
        FusionPressureState::UNKNOWN;

    FusionTemperatureState temperature =
        FusionTemperatureState::UNKNOWN;

    FusionRespirationState respiration =
        FusionRespirationState::UNKNOWN;


    // --------------------------------------------------------
    // Evidence summary
    // --------------------------------------------------------

    FusionEvidenceSummary evidence;


    // --------------------------------------------------------
    // Final system decision
    // --------------------------------------------------------

    FusionLevel level =
        FusionLevel::WATCH;


    // 0.0 ... 1.0
    //
    // This will represent confidence in the FUSION DECISION,
    // not a medical probability.
    float confidence = 0.0f;


    // --------------------------------------------------------
    // Outputs to other subsystems
    // --------------------------------------------------------

    bool triggerCamera = false;

    bool triggerAlert = false;


    // --------------------------------------------------------
    // Timing
    // --------------------------------------------------------

    unsigned long lastUpdateMillis = 0;
};


// ============================================================
// FUSION ENGINE DECLARATION
//
// Step 5.1 only defines the API.
// Step 5.2 will implement this in Fusion.cpp.
// ============================================================

class FusionEngine
{
public:
    FusionEngine();

    void begin();

    void update(
        const FusionInput& input
    );

    const FusionReading&
    getReading() const;


    // --------------------------------------------------------
    // Text helpers for Serial dashboard
    // --------------------------------------------------------

    static const char*
    getSensorHealthText(
        FusionSensorHealth health
    );

    static const char*
    getOccupancyText(
        FusionOccupancyState state
    );

    static const char*
    getMotionText(
        FusionMotionState state
    );

    static const char*
    getVitalsText(
        FusionVitalsState state
    );

    static const char*
    getPressureText(
        FusionPressureState state
    );

    static const char*
    getTemperatureText(
        FusionTemperatureState state
    );

    static const char*
    getRespirationText(
        FusionRespirationState state
    );

    static const char*
    getLevelText(
        FusionLevel level
    );


private:
    static constexpr unsigned long
        WARNING_PERSIST_MS = 4000UL;

    static constexpr unsigned long
        EMERGENCY_PERSIST_MS = 2500UL;

    static constexpr unsigned long
        CLEAR_STABLE_MS = 3000UL;

    // MPU physical-motion qualification validated on 2026-08-23.
    // Three consecutive fresh samples at ~80 Hz = ~37.5 ms.
    // Physical motion must exist before IF/OCSVM road-domain
    // output is allowed to affect vehicle-motion context.
    static constexpr uint8_t
        MPU_MOTION_PERSIST_SAMPLES = 3U;

    static constexpr float
        MPU_ACTIVE_ACCEL_G = 0.040f;

    static constexpr float
        MPU_ACTIVE_GYRO_DPS = 5.0f;

    static constexpr float
        MPU_STRONG_ACCEL_G = 0.20f;

    static constexpr float
        MPU_STRONG_GYRO_DPS = 35.0f;

    FusionReading reading;

    unsigned long warningCandidateStartMillis = 0UL;

    unsigned long emergencyCandidateStartMillis = 0UL;

    unsigned long clearStateStartMillis = 0UL;

    FusionLevel previousLevel =
        FusionLevel::WATCH;

    // Camera transaction state - Step 5.9.4.
    uint32_t lastCameraRequestId = 0;
    uint32_t lastCameraResultId = 0;
    bool cameraAbnormalLatched = false;

    // MPU motion-persistence state.
    unsigned long lastMpuMotionSampleCount = 0UL;
    uint8_t mpuActivePersistenceSamples = 0U;
    uint8_t mpuStrongPersistenceSamples = 0U;
};