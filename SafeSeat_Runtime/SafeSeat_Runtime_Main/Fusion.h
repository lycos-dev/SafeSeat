#pragma once

#include <Arduino.h>

#include "C1001.h"
#include "MLX.h"
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
// - Piezo and Camera are represented as placeholders for now.
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
    STABLE,
    CONTEXT_CHANGE,
    ANOMALOUS
};


// ============================================================
// RESPIRATION / PIEZO STATE
//
// Piezo runs on another ESP32.
// For now these fields are placeholders.
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
// PIEZO EVIDENCE PLACEHOLDER
//
// This will later be populated from PiezoComm after the
// separate seatbelt ESP32 sends its current runtime result.
// ============================================================

struct PiezoFusionEvidence
{
    bool available = false;
    bool connected = false;
    bool valid = false;

    bool signalQualityValid = false;

    float peakRespirationBPM = NAN;
    float spectralRespirationBPM = NAN;

    bool noBreathTimerExceeded = false;

    ModelEvidence model;

    unsigned long lastUpdateMillis = 0;
};


// ============================================================
// CAMERA VERIFICATION PLACEHOLDER
//
// Fusion requests camera verification.
// The ESP32-CAM remains a separate verification subsystem.
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

    // Optional future class confidence.
    float confidence = 0.0f;

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
// Main Hub sensors are already present.
// Piezo and camera remain placeholders until communication
// is integrated.
// ============================================================

struct FusionInput
{
    C1001FusionInput c1001;
    MLXFusionInput mlx;
    FSRFusionInput fsr;
    MPUFusionInput mpu;

    PiezoFusionEvidence piezo;
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
    FusionReading reading;
};
