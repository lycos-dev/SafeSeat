#include "SafeSeatUatInjection.h"

#include <math.h>

#include "SafeSeatNow.h"
#include "SafeSeatNowProtocol.h"

namespace
{
constexpr unsigned long UAT_FSR_INTERVAL_MS = 220UL;
constexpr unsigned long UAT_MLX_INTERVAL_MS = 250UL;
constexpr unsigned long UAT_MPU_INTERVAL_MS = 13UL; // ~77 Hz, close to trained ~80 Hz domain.
constexpr unsigned long UAT_C1001_COMMAND_INTERVAL_MS = 500UL;
constexpr unsigned long UAT_CAMERA_RESULT_DELAY_MS = 350UL;
constexpr float UAT_FSR_COMMON_SCALE = 26000.0f;
constexpr float UAT_FSR_ACTIVE_THRESHOLD = 0.025f;

// Controlled physical/injected FSR validation thresholds.
// These reflect the observed installed seat behavior: loaded contact points
// are commonly ~20k-26k, while a hard side slump can unload multiple
// opposite-side sensors toward zero. A severe pattern must remain continuous
// for 10 s before it is allowed to become the UAT strong FSR vote.
constexpr float UAT_FSR_UNLOADED_MAX = 5000.0f;
constexpr float UAT_FSR_DOMINANT_BACKREST_MIN = 35000.0f;
constexpr float UAT_FSR_SIDE_BALANCE_MIN = 0.40f;
constexpr uint8_t UAT_FSR_OPPOSITE_UNLOADED_MIN = 2U;
constexpr unsigned long UAT_FSR_CONTROLLED_PERSIST_MS = 10000UL;
constexpr unsigned long UAT_FSR_DYNAMIC_SIDE_MS = 1800UL;

float safeBalanceUat(float left, float right)
{
    const float total = left + right;
    if (total <= 1.0f) return 0.0f;
    return (left - right) / total;
}
}

SafeSeatUatInjection& SafeSeatUatInjection::instance()
{
    static SafeSeatUatInjection injection;
    return injection;
}

float SafeSeatUatInjection::clampPressure(float value)
{
    if (!isfinite(value) || value < 0.0f) return 0.0f;
    if (value > UAT_FSR_COMMON_SCALE) return UAT_FSR_COMMON_SCALE;
    return value;
}

void SafeSeatUatInjection::setInjectFsr(bool enabled)
{
    state.injectFsr = enabled;
    state.revision++;
}

void SafeSeatUatInjection::setInjectMlx(bool enabled)
{
    state.injectMlx = enabled;
    state.revision++;
}

void SafeSeatUatInjection::setInjectMpu(bool enabled)
{
    state.injectMpu = enabled;
    state.revision++;
}

void SafeSeatUatInjection::setInjectC1001(bool enabled)
{
    state.injectC1001 = enabled;
    state.revision++;
}

void SafeSeatUatInjection::setMlxRapidTransitionBypass(bool enabled)
{
    state.mlxRapidTransitionBypass = enabled;
    state.revision++;
}

void SafeSeatUatInjection::setFsrDynamicPattern(bool enabled)
{
    state.fsrDynamicPattern = enabled;
    state.revision++;
}

void SafeSeatUatInjection::setFsrPressure(uint8_t index, float value)
{
    if (index >= NUM_FSR) return;
    state.fsrPressure[index] = clampPressure(value);
    state.revision++;
}

void SafeSeatUatInjection::setMlx(float ambientC, float objectC)
{
    if (isfinite(ambientC)) state.mlxAmbientC = ambientC;
    if (isfinite(objectC)) state.mlxObjectC = objectC;
    state.revision++;
}

void SafeSeatUatInjection::setMpu(float ax, float ay, float az, float gx, float gy, float gz)
{
    if (isfinite(ax)) state.mpuAccelX = ax;
    if (isfinite(ay)) state.mpuAccelY = ay;
    if (isfinite(az)) state.mpuAccelZ = az;
    if (isfinite(gx)) state.mpuGyroX = gx;
    if (isfinite(gy)) state.mpuGyroY = gy;
    if (isfinite(gz)) state.mpuGyroZ = gz;
    state.revision++;
}

void SafeSeatUatInjection::setC1001(bool present, int heartRate, int respiration, int motion, int moveRange)
{
    state.c1001Present = present;
    state.c1001HeartRate = constrain(heartRate, 1, 254);
    state.c1001Respiration = constrain(respiration, 1, 254);
    state.c1001Motion = constrain(motion, -32768, 32767);
    state.c1001MoveRange = constrain(moveRange, -32768, 32767);
    state.revision++;
}

void SafeSeatUatInjection::setCameraMode(SafeSeatUatCameraMode mode)
{
    state.cameraMode = mode;
    state.revision++;
    lastFusionCameraTrigger = false;
    syntheticCameraPending = false;
    syntheticCameraDelivered = false;
}

void SafeSeatUatInjection::resetSyntheticClocks()
{
    lastFsrSampleMillis = 0;
    lastMlxSampleMillis = 0;
    lastMpuSampleMillis = 0;
    syntheticFsrFrame = 0;
    syntheticMlxSample = 0;
    syntheticMpuSample = 0;

    fsrControlledCandidateSinceMillis = 0;
    state.fsrControlledPatternActive = false;
    state.fsrControlledStrongVote = false;
    state.fsrControlledCandidateMs = 0;
    state.fsrControlledSide = 0;

    lastFusionCameraTrigger = false;
    syntheticCameraPending = false;
    syntheticCameraDelivered = false;
    syntheticCameraPendingSince = 0;
}

void SafeSeatUatInjection::start()
{
    state.active = true;
    state.sessionId++;
    if (state.sessionId == 0) state.sessionId = 1;
    state.startedMillis = millis();
    state.revision++;
    c1001DisableRepeatsRemaining = 0;
    lastC1001CommandMillis = 0;
    resetSyntheticClocks();

    Serial.print("[UAT-INJECT] START session=");
    Serial.println(state.sessionId);
}

void SafeSeatUatInjection::stop()
{
    if (!state.active && c1001DisableRepeatsRemaining == 0) return;

    state.active = false;
    state.sessionId++;
    if (state.sessionId == 0) state.sessionId = 1;
    state.revision++;
    c1001DisableRepeatsRemaining = 3;
    lastC1001CommandMillis = 0;
    resetSyntheticClocks();

    Serial.print("[UAT-INJECT] STOP session=");
    Serial.println(state.sessionId);
}

void SafeSeatUatInjection::recomputeFsrDerived(FSRReading &reading) const
{
    reading.backrestLeftTotal = reading.pressure[0] + reading.pressure[1] + reading.pressure[2];
    reading.backrestRightTotal = reading.pressure[3] + reading.pressure[4] + reading.pressure[5];
    reading.backrestTotal = reading.backrestLeftTotal + reading.backrestRightTotal;

    reading.cushionLeft = reading.pressure[6];
    reading.cushionCenter = reading.pressure[7];
    reading.cushionRight = reading.pressure[8];
    reading.cushionTotal = reading.cushionLeft + reading.cushionCenter + reading.cushionRight;
    reading.wholeSeatTotal = reading.backrestTotal + reading.cushionTotal;

    reading.backrestLRBalance = safeBalanceUat(reading.backrestLeftTotal, reading.backrestRightTotal);
    reading.cushionLRBalance = safeBalanceUat(reading.cushionLeft, reading.cushionRight);
    reading.backrestToCushionRatio = reading.backrestTotal / ((reading.cushionTotal > 1.0f) ? reading.cushionTotal : 1.0f);

    reading.activeSensorCount = 0;
    for (uint8_t i = 0; i < NUM_FSR; ++i)
    {
        reading.normalized[i] = reading.pressure[i] / UAT_FSR_COMMON_SCALE;
        if (reading.normalized[i] >= UAT_FSR_ACTIVE_THRESHOLD) reading.activeSensorCount++;
        reading.modelShare[i] = reading.wholeSeatTotal > 1.0f
            ? reading.pressure[i] / reading.wholeSeatTotal
            : 0.0f;
    }

    reading.occupiedByPressure =
        reading.wholeSeatTotal >= 12000.0f
        && reading.activeSensorCount >= 2;
}

void SafeSeatUatInjection::applyFsr(FSRReading &reading)
{
    if (!state.active || !state.injectFsr) return;

    const unsigned long now = millis();
    if (lastFsrSampleMillis == 0 || now - lastFsrSampleMillis >= UAT_FSR_INTERVAL_MS)
    {
        lastFsrSampleMillis = now;
        syntheticFsrFrame++;
    }

    reading.connected = true;
    reading.valid = true;
    reading.status = FSRStatus::READING;
    reading.ads1Connected = true;
    reading.ads2Connected = true;
    reading.baselineValid = true;
    reading.maintenanceActive = false;
    reading.emptyBaselineTracking = false;

    // Manual mode uses exactly the evaluator-entered values.
    // Dynamic mode deliberately imitates the REAL installed seat behavior
    // observed during hard side slumps: one side stays heavily loaded
    // (~20k-26k) while the opposite side loses contact toward zero.
    // It changes sides slowly (~1.8 s) rather than every frame.
    static const float UAT_FSR_DYNAMIC_LEFT[NUM_FSR] = {
        25500.0f, 25000.0f, 22000.0f,
        0.0f, 0.0f, 0.0f,
        26000.0f, 24000.0f, 0.0f
    };
    static const float UAT_FSR_DYNAMIC_RIGHT[NUM_FSR] = {
        0.0f, 0.0f, 0.0f,
        25500.0f, 25000.0f, 22000.0f,
        0.0f, 24000.0f, 26000.0f
    };

    const float *activePattern = state.fsrPressure;
    if (state.fsrDynamicPattern)
    {
        const bool leftPhase =
            ((now / UAT_FSR_DYNAMIC_SIDE_MS) & 1UL) == 0UL;

        activePattern =
            leftPhase
                ? UAT_FSR_DYNAMIC_LEFT
                : UAT_FSR_DYNAMIC_RIGHT;
    }

    for (uint8_t i = 0; i < NUM_FSR; ++i)
    {
        const float p = clampPressure(activePattern[i]);
        reading.pressure[i] = p;
        reading.raw[i] = p;
        reading.baseline[i] = 0.0f;
        reading.electricalRaw[i] = p;
        reading.electricalBaseline[i] = 0.0f;
    }

    recomputeFsrDerived(reading);
    reading.sampleCount = syntheticFsrFrame;
    reading.lastSampleMillis = lastFsrSampleMillis;
    reading.actualSamplingRateHz = 1000.0f / static_cast<float>(UAT_FSR_INTERVAL_MS);
}


void SafeSeatUatInjection::updateControlledFsrObservation(const FSRReading &reading)
{
    const unsigned long now = millis();

    if (!state.active
        || !reading.connected
        || !reading.valid
        || !reading.occupiedByPressure)
    {
        fsrControlledCandidateSinceMillis = 0;
        state.fsrControlledPatternActive = false;
        state.fsrControlledStrongVote = false;
        state.fsrControlledCandidateMs = 0;
        state.fsrControlledSide = 0;
        return;
    }

    uint8_t leftUnloaded = 0;
    uint8_t rightUnloaded = 0;

    for (uint8_t i = 0; i < 3; ++i)
    {
        if (reading.pressure[i] <= UAT_FSR_UNLOADED_MAX) leftUnloaded++;
        if (reading.pressure[i + 3] <= UAT_FSR_UNLOADED_MAX) rightUnloaded++;
    }

    const bool hardLeft =
        reading.backrestLeftTotal >= UAT_FSR_DOMINANT_BACKREST_MIN
        && rightUnloaded >= UAT_FSR_OPPOSITE_UNLOADED_MIN
        && reading.backrestLRBalance >= UAT_FSR_SIDE_BALANCE_MIN;

    const bool hardRight =
        reading.backrestRightTotal >= UAT_FSR_DOMINANT_BACKREST_MIN
        && leftUnloaded >= UAT_FSR_OPPOSITE_UNLOADED_MIN
        && reading.backrestLRBalance <= -UAT_FSR_SIDE_BALANCE_MIN;

    const bool patternActive = hardLeft || hardRight;

    state.fsrControlledPatternActive = patternActive;
    state.fsrControlledSide =
        hardLeft && !hardRight
            ? -1
            : hardRight && !hardLeft
                ? 1
                : 0;

    if (!patternActive)
    {
        fsrControlledCandidateSinceMillis = 0;
        state.fsrControlledCandidateMs = 0;
        state.fsrControlledStrongVote = false;
        return;
    }

    if (fsrControlledCandidateSinceMillis == 0)
    {
        fsrControlledCandidateSinceMillis = now;
    }

    state.fsrControlledCandidateMs =
        static_cast<uint32_t>(
            now - fsrControlledCandidateSinceMillis
        );

    state.fsrControlledStrongVote =
        state.fsrControlledCandidateMs >= UAT_FSR_CONTROLLED_PERSIST_MS;
}

void SafeSeatUatInjection::applyMlx(MLXReading &reading)
{
    if (!state.active || !state.injectMlx) return;

    const unsigned long now = millis();
    bool newSample = false;
    if (lastMlxSampleMillis == 0 || now - lastMlxSampleMillis >= UAT_MLX_INTERVAL_MS)
    {
        lastMlxSampleMillis = now;
        syntheticMlxSample++;
        newSample = true;
    }

    reading.connected = true;
    reading.valid = true;
    reading.rawAmbientC = state.mlxAmbientC;
    reading.rawObjectC = state.mlxObjectC;
    reading.filteredAmbientC = state.mlxAmbientC;
    reading.filteredObjectC = state.mlxObjectC;
    reading.objectMinusAmbientC = state.mlxObjectC - state.mlxAmbientC;
    reading.currentSampleAccepted = newSample;
    reading.acceptedSampleCount = syntheticMlxSample;
    reading.status = MLXStatus::TRUSTED;
}

void SafeSeatUatInjection::applyMpu(MPUReading &reading)
{
    if (!state.active || !state.injectMpu) return;

    const unsigned long now = millis();
    if (lastMpuSampleMillis == 0 || now - lastMpuSampleMillis >= UAT_MPU_INTERVAL_MS)
    {
        lastMpuSampleMillis = now;
        syntheticMpuSample++;
    }

    reading.connected = true;
    reading.valid = true;
    reading.status = MPUStatus::READY;
    reading.accelX = state.mpuAccelX;
    reading.accelY = state.mpuAccelY;
    reading.accelZ = state.mpuAccelZ;
    reading.gyroX = state.mpuGyroX;
    reading.gyroY = state.mpuGyroY;
    reading.gyroZ = state.mpuGyroZ;
    reading.accelMagnitude = sqrtf(
        reading.accelX * reading.accelX
        + reading.accelY * reading.accelY
        + reading.accelZ * reading.accelZ
    );
    reading.gyroMagnitude = sqrtf(
        reading.gyroX * reading.gyroX
        + reading.gyroY * reading.gyroY
        + reading.gyroZ * reading.gyroZ
    );
    reading.dynamicAcceleration = fabsf(reading.accelMagnitude - 1.0f);
    reading.sampleCount = syntheticMpuSample;
    reading.actualSamplingRateHz = 1000.0f / static_cast<float>(UAT_MPU_INTERVAL_MS);
}

void SafeSeatUatInjection::serviceC1001Transport()
{
    const bool needDisableRepeat = c1001DisableRepeatsRemaining > 0;
    const bool shouldSend =
        state.active
        || needDisableRepeat;

    if (!shouldSend) return;

    const unsigned long now = millis();
    if (lastC1001CommandMillis != 0
        && now - lastC1001CommandMillis < UAT_C1001_COMMAND_INTERVAL_MS)
    {
        return;
    }
    lastC1001CommandMillis = now;

    C1001UatCommandPacket packet;
    packet.sessionId = state.sessionId;
    packet.sequence = ++c1001CommandSequence;
    packet.flags = 0;

    if (state.active && state.injectC1001)
    {
        packet.flags |= C1001_UAT_FLAG_ENABLED;
        if (state.c1001Present) packet.flags |= C1001_UAT_FLAG_PRESENT;
        packet.heartRate = static_cast<int16_t>(state.c1001HeartRate);
        packet.respiration = static_cast<int16_t>(state.c1001Respiration);
        packet.motion = static_cast<int16_t>(state.c1001Motion);
        packet.moveRange = static_cast<int16_t>(state.c1001MoveRange);
    }

    packet.checksum = 0;
    packet.checksum = c1001UatCommandChecksum(packet);

    SafeSeatNow::instance().sendC1001UatCommand(packet);

    if (!(state.active && state.injectC1001) && c1001DisableRepeatsRemaining > 0)
    {
        c1001DisableRepeatsRemaining--;
    }
}

bool SafeSeatUatInjection::usingSyntheticCamera() const
{
    return state.active
        && state.cameraMode != SafeSeatUatCameraMode::REAL_CAMERA;
}

CameraFusionEvidence SafeSeatUatInjection::cameraEvidence(const CameraFusionEvidence &realEvidence)
{
    if (!usingSyntheticCamera()) return realEvidence;

    CameraFusionEvidence evidence;
    evidence.available = true;
    evidence.connected = true;

    if (!syntheticCameraPending
        || syntheticCameraDelivered
        || millis() - syntheticCameraPendingSince < UAT_CAMERA_RESULT_DELAY_MS)
    {
        return evidence;
    }

    syntheticCameraDelivered = true;
    evidence.resultValid = true;
    evidence.confidence = 0.99f;
    evidence.requestId = syntheticCameraRequestId;
    evidence.resultId = ++syntheticCameraResultId;
    evidence.lastUpdateMillis = millis();

    if (state.cameraMode == SafeSeatUatCameraMode::INJECT_UPRIGHT)
    {
        evidence.postureNormal = true;
        evidence.postureAbnormal = false;
        evidence.postureClass = static_cast<uint8_t>(CameraPostureClass::UPRIGHT);
    }
    else
    {
        evidence.postureNormal = false;
        evidence.postureAbnormal = true;
        evidence.postureClass = static_cast<uint8_t>(CameraPostureClass::NON_UPRIGHT);
    }

    return evidence;
}

void SafeSeatUatInjection::noteFusionReading(const FusionReading &reading)
{
    if (!usingSyntheticCamera())
    {
        lastFusionCameraTrigger = reading.triggerCamera;
        syntheticCameraPending = false;
        syntheticCameraDelivered = false;
        return;
    }

    if (reading.triggerCamera && !lastFusionCameraTrigger)
    {
        syntheticCameraRequestId++;
        if (syntheticCameraRequestId == 0) syntheticCameraRequestId = 1;
        syntheticCameraPending = true;
        syntheticCameraDelivered = false;
        syntheticCameraPendingSince = millis();

        Serial.print("[UAT-CAMERA] Fusion requested verification; synthetic request=");
        Serial.println(syntheticCameraRequestId);
    }

    if (!reading.triggerCamera && lastFusionCameraTrigger)
    {
        syntheticCameraPending = false;
    }

    lastFusionCameraTrigger = reading.triggerCamera;
}
