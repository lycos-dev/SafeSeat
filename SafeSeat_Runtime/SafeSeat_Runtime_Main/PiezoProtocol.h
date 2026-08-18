#pragma once

#include <stdint.h>
#include <stddef.h>

#include "SafeSeatNowProtocol.h"

// ============================================================
// SAFESEAT PIEZO -> MAIN HUB RESPIRATION SUPPORT PROTOCOL
// STEP 5.9.8.2
//
// The Piezo node no longer sends IF/OCSVM model evidence.
// It sends deterministic mechanical-respiration support only.
// Transport remains ESP-NOW broadcast.
// ============================================================

constexpr uint16_t PIEZO_WIRE_MAGIC = 0x5350u; // "SP"
constexpr uint8_t PIEZO_WIRE_VERSION = 2u;
constexpr uint8_t PIEZO_WIRE_PACKET_SIZE = 40u;

enum PiezoWireFlags : uint16_t
{
    PIEZO_FLAG_SENSOR_VALID             = 1u << 0,
    PIEZO_FLAG_SIGNAL_USABLE            = 1u << 1,
    PIEZO_FLAG_BREATH_TRACKING_READY    = 1u << 2,
    PIEZO_FLAG_RR_VALID                 = 1u << 3,
    PIEZO_FLAG_NO_BREATH_TIMER          = 1u << 4,
    PIEZO_FLAG_BREATH_DETECTED_RECENT   = 1u << 5
};

struct __attribute__((packed)) PiezoWirePacket
{
    uint16_t magic = PIEZO_WIRE_MAGIC;
    uint8_t version = PIEZO_WIRE_VERSION;
    uint8_t packetSize = PIEZO_WIRE_PACKET_SIZE;

    uint32_t sequence = 0;
    uint32_t senderMillis = 0;
    uint16_t flags = 0;

    float respirationBPM = 0.0f;
    float respirationWave = 0.0f;
    float actualSamplingRateHz = 0.0f;

    uint32_t totalBreaths = 0;
    uint32_t noBreathDurationMs = 0;
    uint32_t sampleCount = 0;

    uint16_t checksum = 0;
};

static_assert(
    sizeof(PiezoWirePacket) == PIEZO_WIRE_PACKET_SIZE,
    "Unexpected PiezoWirePacket size"
);

inline uint16_t piezoPacketChecksum(
    const PiezoWirePacket &packet
)
{
    return safeSeatCRC16(
        reinterpret_cast<const uint8_t *>(&packet),
        offsetof(PiezoWirePacket, checksum)
    );
}
