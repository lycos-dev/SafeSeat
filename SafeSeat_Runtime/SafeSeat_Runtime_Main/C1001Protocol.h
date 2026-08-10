#pragma once

#include <stdint.h>
#include <stddef.h>

#include "SafeSeatNowProtocol.h"

// ============================================================
// SAFESEAT C1001 NODE -> MAIN HUB PROTOCOL
//
// The remote node performs BOTH C1001 acquisition/filtering and
// the 64-feature IF + OCSVM inference locally. The Main Hub only
// receives the resulting sensor/model evidence.
// ============================================================

constexpr uint16_t C1001_WIRE_MAGIC = 0x5343u; // "SC"
constexpr uint8_t C1001_WIRE_VERSION = 1u;
constexpr uint8_t C1001_WIRE_PACKET_SIZE = 62u;

enum C1001WireFlags : uint16_t
{
    C1001_FLAG_SENSOR_CONNECTED   = 1u << 0,
    C1001_FLAG_PRESENT            = 1u << 1,
    C1001_FLAG_WARMED_UP          = 1u << 2,
    C1001_FLAG_TRUSTED_VITALS     = 1u << 3,
    C1001_FLAG_MOTION_ARTIFACT    = 1u << 4,
    C1001_FLAG_VALID_RR           = 1u << 5,
    C1001_FLAG_VALID_HR           = 1u << 6,
    C1001_FLAG_VALID_PAIR         = 1u << 7,
    C1001_FLAG_MODEL_AVAILABLE    = 1u << 8,
    C1001_FLAG_MODEL_VALID        = 1u << 9,
    C1001_FLAG_IF_ANOMALY         = 1u << 10,
    C1001_FLAG_SVM_ANOMALY        = 1u << 11,
    C1001_FLAG_BOTH_ANOMALY       = 1u << 12,
    C1001_FLAG_EITHER_ANOMALY     = 1u << 13
};

struct __attribute__((packed)) C1001WirePacket
{
    uint16_t magic = C1001_WIRE_MAGIC;
    uint8_t version = C1001_WIRE_VERSION;
    uint8_t packetSize = C1001_WIRE_PACKET_SIZE;

    uint32_t sequence = 0;
    uint32_t senderMillis = 0;
    uint16_t flags = 0;

    uint8_t sensorStatus = 0;
    uint8_t mlStatus = 0;

    int16_t motion = -1;
    int16_t moveRange = -1;
    int16_t rawRespiration = 0;
    int16_t rawHeartRate = 0;
    int16_t medianRespiration = 0;
    int16_t medianHeartRate = 0;

    float filteredRespiration = 0.0f;
    float filteredHeartRate = 0.0f;

    uint16_t warmupRemainingSeconds = 0;
    uint16_t windowSamplesCollected = 0;
    uint16_t samplesUntilNextInference = 0;
    uint16_t windowSamplesRequired = 0;

    uint32_t sampleSequence = 0;
    uint32_t windowsEvaluated = 0;

    float isolationForestDecision = 0.0f;
    float oneClassSVMDecision = 0.0f;

    uint16_t checksum = 0;
};

static_assert(
    sizeof(C1001WirePacket) == C1001_WIRE_PACKET_SIZE,
    "Unexpected C1001WirePacket size"
);

inline uint16_t c1001PacketChecksum(
    const C1001WirePacket &packet
)
{
    return safeSeatCRC16(
        reinterpret_cast<const uint8_t *>(&packet),
        offsetof(C1001WirePacket, checksum)
    );
}
