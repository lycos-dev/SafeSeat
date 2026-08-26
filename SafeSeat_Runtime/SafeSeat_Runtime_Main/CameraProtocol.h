#pragma once

#include <stdint.h>
#include <stddef.h>
#include "SafeSeatNowProtocol.h"

// SafeSeat Camera ESP-NOW protocol V2.
// The camera is verification-only. It reports NORMAL/non-upright
// confirmation semantics, not a medical diagnosis or 5-way posture class.

constexpr uint16_t CAMERA_STATUS_MAGIC  = 0x4353u; // CS
constexpr uint16_t CAMERA_COMMAND_MAGIC = 0x434Du; // CM
constexpr uint16_t CAMERA_RESULT_MAGIC  = 0x4352u; // CR
constexpr uint8_t CAMERA_WIRE_VERSION = 2u;
constexpr uint8_t CAMERA_STATUS_PACKET_SIZE = 30u;
constexpr uint8_t CAMERA_COMMAND_PACKET_SIZE = 18u;
constexpr uint8_t CAMERA_RESULT_PACKET_SIZE = 36u;

enum class CameraCommandType : uint8_t
{
    PING = 1,
    CALIBRATE_UPRIGHT = 2,
    VERIFY_POSTURE = 3,
    CANCEL_VERIFY = 4,
    RESET_SESSION = 5
};

// Kept as CameraPostureClass for compatibility with existing Fusion/API fields.
// Values now describe verification decisions rather than directional classes.
enum class CameraPostureClass : uint8_t
{
    UPRIGHT = 0,
    DEVIATION_PENDING = 1,
    NON_UPRIGHT = 2,
    NOT_READY = 3,
    UNKNOWN = 255
};

constexpr uint8_t CAMERA_STATUS_MODEL_READY          = 1u << 0;
constexpr uint8_t CAMERA_STATUS_CAMERA_READY         = 1u << 1;
constexpr uint8_t CAMERA_STATUS_PSRAM_READY          = 1u << 2;
constexpr uint8_t CAMERA_STATUS_BUSY                 = 1u << 3;
constexpr uint8_t CAMERA_STATUS_BASELINE_READY       = 1u << 4;
constexpr uint8_t CAMERA_STATUS_CALIBRATING          = 1u << 5;
constexpr uint8_t CAMERA_STATUS_SESSION_ACTIVE       = 1u << 6;
constexpr uint8_t CAMERA_STATUS_BASELINE_PROVISIONAL = 1u << 7;

constexpr uint8_t CAMERA_RESULT_VALID                = 1u << 0;
constexpr uint8_t CAMERA_RESULT_CAPTURE_ERROR        = 1u << 1;
constexpr uint8_t CAMERA_RESULT_INFERENCE_ERROR      = 1u << 2;
constexpr uint8_t CAMERA_RESULT_BASELINE_PROVISIONAL = 1u << 3;

struct __attribute__((packed)) CameraStatusPacket
{
    uint16_t magic = CAMERA_STATUS_MAGIC;
    uint8_t version = CAMERA_WIRE_VERSION;
    uint8_t packetSize = CAMERA_STATUS_PACKET_SIZE;
    uint32_t sequence = 0;
    uint32_t sessionId = 0;
    uint32_t lastHandledRequestId = 0;
    uint32_t lastInferenceMillis = 0;
    uint32_t freeHeapBytes = 0;
    uint8_t flags = 0;
    uint8_t channel = 0;
    uint8_t calibrationCount = 0;
    uint8_t calibrationTarget = 5;
    uint16_t checksum = 0;
};

struct __attribute__((packed)) CameraCommandPacket
{
    uint16_t magic = CAMERA_COMMAND_MAGIC;
    uint8_t version = CAMERA_WIRE_VERSION;
    uint8_t packetSize = CAMERA_COMMAND_PACKET_SIZE;
    uint32_t requestId = 0;
    uint32_t sessionId = 0;
    uint8_t command = static_cast<uint8_t>(CameraCommandType::PING);
    uint8_t flags = 0;
    uint16_t reserved = 0;
    uint16_t checksum = 0;
};

struct __attribute__((packed)) CameraResultPacket
{
    uint16_t magic = CAMERA_RESULT_MAGIC;
    uint8_t version = CAMERA_WIRE_VERSION;
    uint8_t packetSize = CAMERA_RESULT_PACKET_SIZE;
    uint32_t requestId = 0;
    uint32_t sessionId = 0;
    uint32_t sequence = 0;
    uint8_t postureClass = static_cast<uint8_t>(CameraPostureClass::UNKNOWN);
    uint8_t rawState = 0;
    uint8_t flags = 0;
    uint8_t validFrames = 0;
    uint16_t confidenceMilli = 0;
    float ifScore = 0.0f;
    float ocsvmScore = 0.0f;
    uint32_t inferenceMillis = 0;
    uint16_t checksum = 0;
};

static_assert(sizeof(CameraStatusPacket) == 30, "Unexpected CameraStatusPacket size");
static_assert(sizeof(CameraCommandPacket) == 18, "Unexpected CameraCommandPacket size");
static_assert(sizeof(CameraResultPacket) == 36, "Unexpected CameraResultPacket size");

inline uint16_t cameraStatusChecksum(const CameraStatusPacket &packet)
{
    return safeSeatCRC16(reinterpret_cast<const uint8_t *>(&packet), offsetof(CameraStatusPacket, checksum));
}
inline uint16_t cameraCommandChecksum(const CameraCommandPacket &packet)
{
    return safeSeatCRC16(reinterpret_cast<const uint8_t *>(&packet), offsetof(CameraCommandPacket, checksum));
}
inline uint16_t cameraResultChecksum(const CameraResultPacket &packet)
{
    return safeSeatCRC16(reinterpret_cast<const uint8_t *>(&packet), offsetof(CameraResultPacket, checksum));
}

inline bool cameraPostureIsNormal(CameraPostureClass posture)
{
    return posture == CameraPostureClass::UPRIGHT;
}
inline bool cameraPostureIsAbnormal(CameraPostureClass posture)
{
    return posture == CameraPostureClass::NON_UPRIGHT;
}
inline bool cameraPostureIsFinal(CameraPostureClass posture)
{
    return cameraPostureIsNormal(posture) || cameraPostureIsAbnormal(posture);
}
inline const char *cameraPostureText(CameraPostureClass posture)
{
    switch (posture)
    {
        case CameraPostureClass::UPRIGHT: return "UPRIGHT CONFIRMED";
        case CameraPostureClass::DEVIATION_PENDING: return "DEVIATION PENDING";
        case CameraPostureClass::NON_UPRIGHT: return "NON-UPRIGHT CONFIRMED";
        case CameraPostureClass::NOT_READY: return "NOT READY";
        default: return "UNKNOWN";
    }
}
