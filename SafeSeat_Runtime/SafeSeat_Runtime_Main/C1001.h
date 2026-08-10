#pragma once

#include <Arduino.h>

// ============================================================
// C1001 SHARED READING TYPES - MAIN HUB
//
// Step 5.8 moves the physical C1001 and its ML pipeline to a
// dedicated ESP32. The Main Hub keeps only these lightweight
// data types so Fusion can consume the remote evidence exactly
// as before.
// ============================================================

enum class C1001Status
{
    DISCONNECTED,
    NO_OCCUPANT,
    WAITING_FOR_VITALS,
    WARMING_UP,
    INVALID_VITALS,
    STRONG_MOTION,
    MOTION_RECOVERY,
    MODERATE_MOTION,
    COLLECTING_FILTER_SAMPLES,
    SPIKE_REJECTED,
    FILTER_INITIALIZED,
    TRUSTED,
    SUSTAINED_CHANGE,
    HOLDING_LAST_VALUE
};

struct C1001Reading
{
    bool connected = false;
    bool present = false;

    int motion = -1;
    int moveRange = -1;

    int rawRespiration = 0;
    int rawHeartRate = 0;

    uint32_t sampleSequence = 0;
    unsigned long sampleTimestampMillis = 0;

    int medianRespiration = 0;
    int medianHeartRate = 0;

    float filteredRespiration = NAN;
    float filteredHeartRate = NAN;

    bool validRespiration = false;
    bool validHeartRate = false;
    bool validPair = false;
    bool warmedUp = false;
    bool trustedVitalsAvailable = false;
    bool motionArtifactActive = false;

    int recoveryStableCount = 0;
    int cleanSampleCount = 0;

    unsigned long warmupRemainingSeconds = 0;

    C1001Status status = C1001Status::DISCONNECTED;
};

inline const char* c1001StatusText(C1001Status status)
{
    switch (status)
    {
        case C1001Status::DISCONNECTED: return "DISCONNECTED";
        case C1001Status::NO_OCCUPANT: return "NO OCCUPANT";
        case C1001Status::WAITING_FOR_VITALS: return "WAITING FOR VITALS";
        case C1001Status::WARMING_UP: return "WARMING UP";
        case C1001Status::INVALID_VITALS: return "INVALID VITALS";
        case C1001Status::STRONG_MOTION: return "STRONG MOTION";
        case C1001Status::MOTION_RECOVERY: return "MOTION RECOVERY";
        case C1001Status::MODERATE_MOTION: return "MODERATE MOTION";
        case C1001Status::COLLECTING_FILTER_SAMPLES: return "COLLECTING FILTER SAMPLES";
        case C1001Status::SPIKE_REJECTED: return "SPIKE REJECTED";
        case C1001Status::FILTER_INITIALIZED: return "FILTER INITIALIZED";
        case C1001Status::TRUSTED: return "TRUSTED";
        case C1001Status::SUSTAINED_CHANGE: return "SUSTAINED CHANGE";
        case C1001Status::HOLDING_LAST_VALUE: return "HOLDING LAST VALUE";
        default: return "UNKNOWN";
    }
}
