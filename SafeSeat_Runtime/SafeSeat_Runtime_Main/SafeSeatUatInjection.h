#pragma once

#include <Arduino.h>

#include "Fusion.h"
#include "CameraProtocol.h"

// ============================================================
// SAFESEAT UAT SENSOR-VALUE INJECTION
//
// Research / validation only. This module never writes a Fusion level.
// It substitutes selected sensor-domain values before their normal model
// wrappers run, then lets the production Fusion engine decide the result.
//
// Camera remains verification-only and event-triggered. A synthetic camera
// result is released ONLY after Fusion has requested camera verification.
// ============================================================

enum class SafeSeatUatCameraMode : uint8_t
{
    REAL_CAMERA = 0,
    INJECT_UPRIGHT = 1,
    INJECT_NON_UPRIGHT = 2
};

struct SafeSeatUatInjectionState
{
    bool active = false;
    uint32_t sessionId = 0;
    uint32_t revision = 0;
    unsigned long startedMillis = 0;

    bool injectFsr = true;
    bool injectMlx = true;
    bool injectMpu = false;
    bool injectC1001 = true;

    float fsrPressure[NUM_FSR] = {
        3200.0f, 3400.0f, 3100.0f,
        3000.0f, 3300.0f, 3150.0f,
        3500.0f, 3450.0f, 3550.0f
    };

    float mlxAmbientC = 29.0f;
    float mlxObjectC = 34.0f;

    float mpuAccelX = 0.0f;
    float mpuAccelY = 0.0f;
    float mpuAccelZ = 1.0f;
    float mpuGyroX = 0.0f;
    float mpuGyroY = 0.0f;
    float mpuGyroZ = 0.0f;

    bool c1001Present = true;
    int c1001HeartRate = 78;
    int c1001Respiration = 16;
    int c1001Motion = 0;
    int c1001MoveRange = 1;

    SafeSeatUatCameraMode cameraMode = SafeSeatUatCameraMode::REAL_CAMERA;
};

class SafeSeatUatInjection
{
public:
    static SafeSeatUatInjection& instance();

    const SafeSeatUatInjectionState& getState() const { return state; }

    void setInjectFsr(bool enabled);
    void setInjectMlx(bool enabled);
    void setInjectMpu(bool enabled);
    void setInjectC1001(bool enabled);

    void setFsrPressure(uint8_t index, float value);
    void setMlx(float ambientC, float objectC);
    void setMpu(float ax, float ay, float az, float gx, float gy, float gz);
    void setC1001(bool present, int heartRate, int respiration, int motion, int moveRange);
    void setCameraMode(SafeSeatUatCameraMode mode);

    void start();
    void stop();

    // Called by the Main Hub loop before ML/model wrappers consume readings.
    void applyFsr(FSRReading &reading);
    void applyMlx(MLXReading &reading);
    void applyMpu(MPUReading &reading);

    // Sends the C1001 value-injection command to the dedicated C1001 node.
    // The remote node still runs the real C1001 IF + OCSVM model.
    void serviceC1001Transport();

    // Synthetic posture verification is event-triggered. This method returns
    // real camera evidence unless a UAT posture mode is selected AND Fusion
    // has already requested verification.
    CameraFusionEvidence cameraEvidence(const CameraFusionEvidence &realEvidence);
    void noteFusionReading(const FusionReading &reading);

    bool usingSyntheticCamera() const;

private:
    SafeSeatUatInjection() = default;

    SafeSeatUatInjectionState state;

    unsigned long lastFsrSampleMillis = 0;
    unsigned long lastMlxSampleMillis = 0;
    unsigned long lastMpuSampleMillis = 0;
    uint32_t syntheticFsrFrame = 0;
    uint32_t syntheticMlxSample = 0;
    uint32_t syntheticMpuSample = 0;

    unsigned long lastC1001CommandMillis = 0;
    uint32_t c1001CommandSequence = 0;
    uint8_t c1001DisableRepeatsRemaining = 0;

    bool lastFusionCameraTrigger = false;
    bool syntheticCameraPending = false;
    bool syntheticCameraDelivered = false;
    unsigned long syntheticCameraPendingSince = 0;
    uint32_t syntheticCameraRequestId = 0;
    uint32_t syntheticCameraResultId = 0;

    static float clampPressure(float value);
    void resetSyntheticClocks();
    void recomputeFsrDerived(FSRReading &reading) const;
};
