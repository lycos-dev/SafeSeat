#pragma once

#include <stdint.h>
#include <stddef.h>

#include "SafeSeatNowProtocol.h"

// ============================================================
// SAFESEAT ESP32-CAM ESP-NOW PROTOCOL - STEP 5.9.4
//
// Main Hub -> ESP32-CAM:
//   CameraTriggerPacket
//
// ESP32-CAM -> Main Hub:
//   CameraStatusPacket
//   CameraResultPacket
//
// Camera result classes intentionally use the same index order as
// the accepted Step 5.9.2 / 5.9.3 model.
// ============================================================

constexpr uint16_t CAMERA_STATUS_MAGIC  = 0x4353u; // "CS"
constexpr uint16_t CAMERA_TRIGGER_MAGIC = 0x4354u; // "CT"
constexpr uint16_t CAMERA_RESULT_MAGIC  = 0x4352u; // "CR"
constexpr uint8_t CAMERA_WIRE_VERSION = 1u;
constexpr uint8_t CAMERA_STATUS_PACKET_SIZE = 26u;
constexpr uint8_t CAMERA_TRIGGER_PACKET_SIZE = 14u;
constexpr uint8_t CAMERA_RESULT_PACKET_SIZE = 24u;

enum class CameraPostureClass : uint8_t
{
    LEANING_BACKWARD = 0,
    LEANING_LEFT = 1,
    LEANING_RIGHT = 2,
    UPRIGHT = 3,
    LEANING_FORWARD = 4,
    UNKNOWN = 255
};

constexpr uint8_t CAMERA_STATUS_MODEL_READY   = 1u << 0;
constexpr uint8_t CAMERA_STATUS_CAMERA_READY  = 1u << 1;
constexpr uint8_t CAMERA_STATUS_PSRAM_READY   = 1u << 2;
constexpr uint8_t CAMERA_STATUS_BUSY          = 1u << 3;

constexpr uint8_t CAMERA_RESULT_VALID          = 1u << 0;
constexpr uint8_t CAMERA_RESULT_CAPTURE_ERROR  = 1u << 1;
constexpr uint8_t CAMERA_RESULT_INFERENCE_ERROR= 1u << 2;

struct __attribute__((packed)) CameraStatusPacket
{
    uint16_t magic = CAMERA_STATUS_MAGIC;
    uint8_t version = CAMERA_WIRE_VERSION;
    uint8_t packetSize = CAMERA_STATUS_PACKET_SIZE;
    uint32_t sequence = 0;
    uint8_t flags = 0;
    uint8_t channel = 0;
    uint16_t reserved = 0;
    uint32_t lastInferenceMillis = 0;
    uint32_t freeHeapBytes = 0;
    uint32_t lastHandledRequestId = 0;
    uint16_t checksum = 0;
};

struct __attribute__((packed)) CameraTriggerPacket
{
    uint16_t magic = CAMERA_TRIGGER_MAGIC;
    uint8_t version = CAMERA_WIRE_VERSION;
    uint8_t packetSize = CAMERA_TRIGGER_PACKET_SIZE;
    uint32_t requestId = 0;
    uint8_t frameCount = 3;
    uint8_t minValidFrames = 2;
    uint16_t reserved = 0;
    uint16_t checksum = 0;
};

struct __attribute__((packed)) CameraResultPacket
{
    uint16_t magic = CAMERA_RESULT_MAGIC;
    uint8_t version = CAMERA_WIRE_VERSION;
    uint8_t packetSize = CAMERA_RESULT_PACKET_SIZE;
    uint32_t requestId = 0;
    uint32_t sequence = 0;
    uint8_t postureClass = static_cast<uint8_t>(CameraPostureClass::UNKNOWN);
    uint8_t validFrames = 0;
    uint8_t flags = 0;
    uint8_t reserved = 0;
    uint16_t confidenceMilli = 0; // 0..1000
    uint32_t inferenceMillis = 0;
    uint16_t checksum = 0;
};

static_assert(sizeof(CameraStatusPacket) == 26, "Unexpected CameraStatusPacket size");
static_assert(sizeof(CameraTriggerPacket) == 14, "Unexpected CameraTriggerPacket size");
static_assert(sizeof(CameraResultPacket) == 24, "Unexpected CameraResultPacket size");

inline uint16_t cameraStatusChecksum(const CameraStatusPacket &packet)
{
    return safeSeatCRC16(
        reinterpret_cast<const uint8_t *>(&packet),
        offsetof(CameraStatusPacket, checksum)
    );
}

inline uint16_t cameraTriggerChecksum(const CameraTriggerPacket &packet)
{
    return safeSeatCRC16(
        reinterpret_cast<const uint8_t *>(&packet),
        offsetof(CameraTriggerPacket, checksum)
    );
}

inline uint16_t cameraResultChecksum(const CameraResultPacket &packet)
{
    return safeSeatCRC16(
        reinterpret_cast<const uint8_t *>(&packet),
        offsetof(CameraResultPacket, checksum)
    );
}

inline bool cameraPostureIsNormal(CameraPostureClass posture)
{
    return posture == CameraPostureClass::UPRIGHT;
}

inline bool cameraPostureIsAbnormal(CameraPostureClass posture)
{
    return posture == CameraPostureClass::LEANING_BACKWARD
        || posture == CameraPostureClass::LEANING_LEFT
        || posture == CameraPostureClass::LEANING_RIGHT
        || posture == CameraPostureClass::LEANING_FORWARD;
}

inline const char *cameraPostureText(CameraPostureClass posture)
{
    switch (posture)
    {
        case CameraPostureClass::LEANING_BACKWARD: return "LEANING BACKWARD";
        case CameraPostureClass::LEANING_LEFT: return "LEANING LEFT";
        case CameraPostureClass::LEANING_RIGHT: return "LEANING RIGHT";
        case CameraPostureClass::UPRIGHT: return "UPRIGHT";
        case CameraPostureClass::LEANING_FORWARD: return "LEANING FORWARD";
        default: return "UNKNOWN";
    }
}
