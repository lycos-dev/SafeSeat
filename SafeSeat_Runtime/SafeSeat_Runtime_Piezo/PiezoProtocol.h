#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================
// SAFESEAT PIEZO -> MAIN HUB UART PROTOCOL
//
// One-way link:
//   Piezo ESP32 TX GPIO17 -> Main Hub RX GPIO25
//   GND -> GND
//
// The packet carries already-computed Piezo evidence only.
// The Main Hub does not run the Piezo model itself.
// ============================================================

constexpr uint16_t PIEZO_WIRE_MAGIC = 0x5350u;
constexpr uint8_t PIEZO_WIRE_VERSION = 1u;
constexpr uint8_t PIEZO_WIRE_PACKET_SIZE = 40u;

enum PiezoWireFlags : uint16_t
{
    PIEZO_FLAG_SENSOR_VALID          = 1u << 0,
    PIEZO_FLAG_SIGNAL_QUALITY_VALID  = 1u << 1,
    PIEZO_FLAG_MODEL_VALID           = 1u << 2,
    PIEZO_FLAG_IF_ANOMALY            = 1u << 3,
    PIEZO_FLAG_SVM_ANOMALY           = 1u << 4,
    PIEZO_FLAG_BOTH_ANOMALY          = 1u << 5,
    PIEZO_FLAG_EITHER_ANOMALY        = 1u << 6,
    PIEZO_FLAG_NO_BREATH_TIMER       = 1u << 7,
    PIEZO_FLAG_FEATURE_READY         = 1u << 8,
    PIEZO_FLAG_SPECTRAL_RR_VALID     = 1u << 9,
    PIEZO_FLAG_PEAK_RR_VALID         = 1u << 10
};

struct __attribute__((packed)) PiezoWirePacket
{
    uint16_t magic = PIEZO_WIRE_MAGIC;
    uint8_t version = PIEZO_WIRE_VERSION;
    uint8_t packetSize = PIEZO_WIRE_PACKET_SIZE;

    uint32_t sequence = 0;
    uint32_t senderMillis = 0;
    uint16_t flags = 0;

    float peakRespirationBPM = 0.0f;
    float spectralRespirationBPM = 0.0f;
    float isolationForestDecision = 0.0f;
    float oneClassSVMDecision = 0.0f;
    float actualSamplingRateHz = 0.0f;

    uint32_t featureWindowCount = 0;

    uint16_t checksum = 0;
};

static_assert(
    sizeof(PiezoWirePacket) == PIEZO_WIRE_PACKET_SIZE,
    "Unexpected PiezoWirePacket size"
);

inline uint16_t piezoCRC16(
    const uint8_t *data,
    size_t length
)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000u)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

inline uint16_t piezoPacketChecksum(
    const PiezoWirePacket &packet
)
{
    return piezoCRC16(
        reinterpret_cast<const uint8_t *>(&packet),
        offsetof(PiezoWirePacket, checksum)
    );
}
