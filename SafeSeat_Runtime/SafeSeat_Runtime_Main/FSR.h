#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ============================================================
// SAFESEAT FSR ARRAY
// 2026-08-21 runtime-aligned revision
//
// Hardware:
//   ADS1115 #1 0x48:
//     A0 = electrical Backrest FSR1
//     A1 = electrical Backrest FSR2
//     A2 = electrical Backrest FSR3
//     A3 = electrical Backrest FSR4
//
//   ADS1115 #2 0x49:
//     A0 = electrical Backrest FSR5
//     A1 = electrical Backrest FSR6
//     A2 = Cushion FSR1
//     A3 = Cushion FSR2
//
//   ESP32 GPIO34 = Cushion FSR3
//
// IMPORTANT PHYSICAL-ORIENTATION CORRECTION:
// Live validation on 2026-08-21 showed that the installed backrest
// left/right orientation is mirrored relative to the earlier software
// assumption. Electrical FSR4-FSR6 are occupant-left; electrical
// FSR1-FSR3 are occupant-right.
//
// Therefore reading.pressure[] is exposed in the LOGICAL / MODEL order:
//   FSR1-FSR3 = physical LEFT top/middle/bottom
//   FSR4-FSR6 = physical RIGHT top/middle/bottom
// while electricalRaw[] preserves the actual ADC-channel identity.
//
// This keeps the trained ChairPose/TDSD feature topology aligned without
// changing the already-validated electrical wiring.
// ============================================================

enum FSRIndex : uint8_t
{
    BACKREST_FSR1 = 0,  // logical physical LEFT top
    BACKREST_FSR2 = 1,  // logical physical LEFT middle
    BACKREST_FSR3 = 2,  // logical physical LEFT bottom
    BACKREST_FSR4 = 3,  // logical physical RIGHT top
    BACKREST_FSR5 = 4,  // logical physical RIGHT middle
    BACKREST_FSR6 = 5,  // logical physical RIGHT bottom
    CUSHION_FSR1  = 6,  // physical left
    CUSHION_FSR2  = 7,  // physical center
    CUSHION_FSR3  = 8,  // physical right
    FSR_COUNT     = 9
};

// Compatibility alias used by the existing validated FSR ML pipeline.
static constexpr uint8_t NUM_FSR = FSR_COUNT;

enum class FSRStatus
{
    UNINITIALIZED,
    CALIBRATING,
    READING,
    DEGRADED,
    RECOVERING,
    CALIBRATION_FAILED
};

struct FSRReading
{
    bool connected = false;
    bool valid = false;
    FSRStatus status = FSRStatus::UNINITIALIZED;

    bool ads1Connected = false;
    bool ads2Connected = false;
    bool baselineValid = false;
    bool maintenanceActive = false;

    // Raw ADC readings in LOGICAL / MODEL order.
    float raw[FSR_COUNT] = {0};

    // Raw ADC baselines in LOGICAL / MODEL order.
    float baseline[FSR_COUNT] = {0};

    // Baseline-corrected, filtered, ADC-scale-normalized pressure.
    //
    // All channels are converted to one common 0..26000 pressure-unit
    // scale before aggregation. This prevents GPIO34's 12-bit 0..4095
    // range from being compared directly with ADS1115 raw magnitudes.
    //
    // This array is in the SAME logical order expected by SafeSeat's
    // 6-backrest + 3-cushion ChairPose/TDSD runtime representation.
    float pressure[FSR_COUNT] = {0};

    // 0..1 normalized channel loading in logical/model order.
    float normalized[FSR_COUNT] = {0};

    // Per-frame 9-sensor shares. Sum is 1.0 when loaded, otherwise 0.
    // The deployed Step 5.5 FSR ML branch is share-based; absolute
    // pressure magnitude is intentionally not a model input.
    float modelShare[FSR_COUNT] = {0};

    // Actual electrical-channel data, useful for maintenance.
    //
    // Indices:
    // 0 ADS1 A0 = electrical FSR1
    // 1 ADS1 A1 = electrical FSR2
    // 2 ADS1 A2 = electrical FSR3
    // 3 ADS1 A3 = electrical FSR4
    // 4 ADS2 A0 = electrical FSR5
    // 5 ADS2 A1 = electrical FSR6
    // 6 ADS2 A2 = cushion FSR1
    // 7 ADS2 A3 = cushion FSR2
    // 8 GPIO34  = cushion FSR3
    float electricalRaw[FSR_COUNT] = {0};
    float electricalBaseline[FSR_COUNT] = {0};

    float backrestLeftTotal = 0.0f;
    float backrestRightTotal = 0.0f;
    float backrestTotal = 0.0f;

    float cushionLeft = 0.0f;
    float cushionCenter = 0.0f;
    float cushionRight = 0.0f;
    float cushionTotal = 0.0f;

    float wholeSeatTotal = 0.0f;
    float backrestLRBalance = 0.0f;
    float cushionLRBalance = 0.0f;
    float backrestToCushionRatio = 0.0f;

    uint8_t activeSensorCount = 0;

    // Debounced pressure occupancy. This is the ONLY FSR signal
    // that may open the MLX thermal session when C1001 is absent.
    bool occupiedByPressure = false;

    // Diagnostics for empty-seat baseline tracking.
    bool emptyBaselineTracking = false;
    uint8_t occupancyEnterStreak = 0;
    uint8_t occupancyExitStreak = 0;

    uint32_t sampleCount = 0;
    uint32_t recoveryCount = 0;
    unsigned long lastSampleMillis = 0;
    float actualSamplingRateHz = 0.0f;
};

class FSRSensor
{
public:
    FSRSensor();

    bool begin();

    // API retained for compatibility with the earlier Main Hub:
    // fsr.update(occupantPresent);
    void update(bool occupantPresent);

    const FSRReading& getReading() const;

    bool hasValidReading() const;
    const char* getStatusText() const;
    const char* getSensorLabel(int logicalIndex) const;
    const char* getElectricalSource(int logicalIndex) const;

    // Manual maintenance option if you intentionally want a fresh
    // empty-seat baseline without reflashing.
    void forceRecalibration();

    // Prints raw electrical channels, baselines, normalized pressure,
    // and ADS health. Safe to call from a temporary diagnostic path.
    void printMaintenanceDiagnostics(Stream& out) const;

private:
    static constexpr uint8_t ADS1_ADDRESS = 0x48;
    static constexpr uint8_t ADS2_ADDRESS = 0x49;
    static constexpr uint8_t CUSHION_RIGHT_GPIO = 34;

    // Preserve live Step 5.5 timing alignment:
    // ~4.5 completed FSR frames per second.
    static constexpr unsigned long SAMPLE_INTERVAL_MS = 220UL;

    static constexpr unsigned long HEALTH_CHECK_INTERVAL_MS = 2000UL;
    static constexpr unsigned long RECOVERY_COOLDOWN_MS = 2000UL;

    // The ADS gain change exposed a slow empty-seat settling/drift
    // on the cushion channels. Give the hardware more time and use
    // a longer calibration sample set before normal runtime starts.
    static constexpr int CALIBRATION_SAMPLES = 60;
    static constexpr unsigned long CALIBRATION_SETTLE_MS = 4000UL;

    static constexpr float COMMON_PRESSURE_SCALE = 26000.0f;
    // ADS1115 is configured at GAIN_ONE (+/-4.096 V). With the
    // SafeSeat 3.3 V divider, full physical pressure is about
    // 3.3/4.096*32767 = 26400 counts. The runtime intentionally
    // maps ~26000 counts to the common 0..26000 pressure scale.
    static constexpr float ADS_EXPECTED_PRESS_MAX = 26000.0f;
    static constexpr float GPIO_EXPECTED_PRESS_MAX = 4095.0f;

    static constexpr float ADS_DEADBAND_RAW = 55.0f;
    static constexpr float GPIO_DEADBAND_RAW = 18.0f;

    static constexpr float FILTER_ALPHA = 0.35f;

    // Empty-seat adaptive baseline:
    // The previous alpha (0.0007) was far too slow. In the combined
    // run, unloaded cushion channels drifted by thousands of common
    // pressure units and falsely created occupancy. While the seat
    // is NOT pressure-latched and C1001 does not confirm a person,
    // low/moderate per-channel drift is followed quickly.
    static constexpr float EMPTY_BASELINE_TRACK_ALPHA = 0.12f;

    // Never learn a genuine strong press into the baseline.
    // One deliberately pressed FSR can still be displayed/tested
    // without being absorbed as "empty".
    static constexpr float EMPTY_TRACK_CHANNEL_FREEZE_LOAD = 6000.0f;

    static constexpr float ACTIVE_NORMALIZED_THRESHOLD = 0.025f;

    // Occupancy is deliberately much stricter than "some pressure".
    // This prevents foam/preload/electrical drift from starting MLX.
    // With the corrected 0..26000 common scale, normal seated loads
    // observed during combined validation are far above this value.
    static constexpr float OCCUPANCY_ENTER_TOTAL = 12000.0f;
    static constexpr float OCCUPANCY_EXIT_TOTAL = 2500.0f;
    static constexpr uint8_t OCCUPANCY_ACTIVE_SENSOR_MIN = 2;
    static constexpr uint8_t OCCUPANCY_ENTER_FRAMES = 3;

    // Faster but conservative exit:
    // - hard empty, OR
    // - one-sensor residual relaxation, OR
    // - a large sustained drop from the seated peak with backrest released.
    static constexpr uint8_t OCCUPANCY_EXIT_FRAMES = 6;
    static constexpr float OCCUPANCY_RESIDUAL_EXIT_TOTAL = 12000.0f;
    static constexpr uint8_t OCCUPANCY_RESIDUAL_MAX_ACTIVE = 1;
    static constexpr float OCCUPANCY_DROP_EXIT_RATIO = 0.30f;
    static constexpr float OCCUPANCY_DROP_MIN_PEAK = 20000.0f;
    static constexpr float OCCUPANCY_DROP_BACKREST_MAX = 2000.0f;
    static constexpr uint8_t OCCUPANCY_DROP_MAX_ACTIVE = 3;

    // An empty-seat baseline this close to ADC full pressure is suspicious.
    static constexpr float SATURATED_BASELINE_FRACTION = 0.92f;

    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

    FSRReading reading;

    bool initialized = false;
    bool baselineReady = false;

    float electricalFilteredLoad[FSR_COUNT] = {0};

    unsigned long lastHealthCheckMillis = 0;
    unsigned long lastRecoveryAttemptMillis = 0;
    unsigned long previousFrameMillis = 0;

    uint8_t suddenZeroStreak = 0;
    float previousWholeSeatTotal = 0.0f;

    bool pressureOccupancyLatched = false;
    uint8_t occupancyEnterStreak = 0;
    uint8_t occupancyExitStreak = 0;
    float occupancyPeakTotal = 0.0f;

    // Anti-rebound re-arm guard after a confirmed seat exit.
    bool occupancyRearmRequired = false;
    uint8_t occupancyRearmStreak = 0;
    static constexpr uint8_t OCCUPANCY_REARM_FRAMES = 6;
    static constexpr uint8_t OCCUPANCY_REARM_MAX_ACTIVE = 1;

    bool i2cProbe(uint8_t address);
    bool initADS1();
    bool initADS2();
    void checkAndRecoverADS();

    bool calibrateEmptySeat();
    bool calibrationLooksValid() const;

    bool acquireElectricalRaw(float out[FSR_COUNT]);
    void mapElectricalToLogical(
        const float electrical[FSR_COUNT],
        float logical[FSR_COUNT]
    ) const;

    float normalizeElectricalChannel(
        uint8_t electricalIndex,
        float raw,
        float baseline
    ) const;

    void processFrame(
        const float electricalRaw[FSR_COUNT],
        bool occupantPresent
    );

    void updateDerivedQuantities();
    void updateSamplingRate(unsigned long now);

    bool frameLooksSuddenlyZero(
        const float electricalRaw[FSR_COUNT],
        bool occupantPresent
    ) const;

    void setStatus(FSRStatus status);
};
