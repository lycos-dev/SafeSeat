#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================
// SAFESEAT ESP-NOW SHARED TRANSPORT
//
// The Main Hub periodically broadcasts this small beacon on its
// CURRENT Wi-Fi channel. Remote SafeSeat nodes (Piezo now,
// ESP32-CAM later) can scan channels until they hear the hub and
// then remain on that channel.
//
// This matters because ESP-NOW and normal Wi-Fi share the ESP32
// radio/channel. If the Main Hub later joins a Wi-Fi network and
// its channel changes, a remote node can rediscover it without a
// hard-coded MAC address.
// ============================================================

constexpr uint16_t SAFESEAT_HUB_BEACON_MAGIC = 0x5348u; // "SH"
constexpr uint8_t SAFESEAT_NOW_PROTOCOL_VERSION = 1u;
constexpr uint8_t SAFESEAT_HUB_BEACON_SIZE = 8u;

inline uint16_t safeSeatCRC16(
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

struct __attribute__((packed)) SafeSeatHubBeacon
{
    uint16_t magic = SAFESEAT_HUB_BEACON_MAGIC;
    uint8_t version = SAFESEAT_NOW_PROTOCOL_VERSION;
    uint8_t packetSize = SAFESEAT_HUB_BEACON_SIZE;
    uint8_t channel = 0;
    uint8_t reserved = 0;
    uint16_t checksum = 0;
};

static_assert(
    sizeof(SafeSeatHubBeacon) == SAFESEAT_HUB_BEACON_SIZE,
    "Unexpected SafeSeatHubBeacon size"
);

inline uint16_t safeSeatHubBeaconChecksum(
    const SafeSeatHubBeacon &beacon
)
{
    return safeSeatCRC16(
        reinterpret_cast<const uint8_t *>(&beacon),
        offsetof(SafeSeatHubBeacon, checksum)
    );
}
