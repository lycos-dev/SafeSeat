#pragma once
#include <cstddef>
#include <cstdint>

constexpr uint16_t CAMERA_STATUS_MAGIC  = 0x4353u;
constexpr uint16_t CAMERA_COMMAND_MAGIC = 0x434Du;
constexpr uint16_t CAMERA_RESULT_MAGIC  = 0x4352u;
constexpr uint8_t CAMERA_WIRE_VERSION = 2u;
constexpr uint8_t CAMERA_STATUS_PACKET_SIZE = 30u;
constexpr uint8_t CAMERA_COMMAND_PACKET_SIZE = 18u;
constexpr uint8_t CAMERA_RESULT_PACKET_SIZE = 36u;

enum class CameraCommandType : uint8_t { PING=1, CALIBRATE_UPRIGHT=2, VERIFY_POSTURE=3, CANCEL_VERIFY=4, RESET_SESSION=5 };
enum class CameraPostureClass : uint8_t { UPRIGHT=0, DEVIATION_PENDING=1, NON_UPRIGHT=2, NOT_READY=3, UNKNOWN=255 };
constexpr uint8_t CAMERA_STATUS_MODEL_READY=1u<<0, CAMERA_STATUS_CAMERA_READY=1u<<1, CAMERA_STATUS_PSRAM_READY=1u<<2, CAMERA_STATUS_BUSY=1u<<3, CAMERA_STATUS_BASELINE_READY=1u<<4, CAMERA_STATUS_CALIBRATING=1u<<5, CAMERA_STATUS_SESSION_ACTIVE=1u<<6, CAMERA_STATUS_BASELINE_PROVISIONAL=1u<<7;
constexpr uint8_t CAMERA_RESULT_VALID=1u<<0, CAMERA_RESULT_CAPTURE_ERROR=1u<<1, CAMERA_RESULT_INFERENCE_ERROR=1u<<2, CAMERA_RESULT_BASELINE_PROVISIONAL=1u<<3;

inline uint16_t camera_crc16(const uint8_t *data, size_t length){uint16_t crc=0xFFFFu;for(size_t i=0;i<length;i++){crc^=static_cast<uint16_t>(data[i])<<8;for(uint8_t bit=0;bit<8;bit++)crc=(crc&0x8000u)?static_cast<uint16_t>((crc<<1)^0x1021u):static_cast<uint16_t>(crc<<1);}return crc;}

struct __attribute__((packed)) CameraStatusPacket{uint16_t magic=CAMERA_STATUS_MAGIC;uint8_t version=CAMERA_WIRE_VERSION;uint8_t packetSize=CAMERA_STATUS_PACKET_SIZE;uint32_t sequence=0;uint32_t sessionId=0;uint32_t lastHandledRequestId=0;uint32_t lastInferenceMillis=0;uint32_t freeHeapBytes=0;uint8_t flags=0;uint8_t channel=0;uint8_t calibrationCount=0;uint8_t calibrationTarget=5;uint16_t checksum=0;};
struct __attribute__((packed)) CameraCommandPacket{uint16_t magic=CAMERA_COMMAND_MAGIC;uint8_t version=CAMERA_WIRE_VERSION;uint8_t packetSize=CAMERA_COMMAND_PACKET_SIZE;uint32_t requestId=0;uint32_t sessionId=0;uint8_t command=static_cast<uint8_t>(CameraCommandType::PING);uint8_t flags=0;uint16_t reserved=0;uint16_t checksum=0;};
struct __attribute__((packed)) CameraResultPacket{uint16_t magic=CAMERA_RESULT_MAGIC;uint8_t version=CAMERA_WIRE_VERSION;uint8_t packetSize=CAMERA_RESULT_PACKET_SIZE;uint32_t requestId=0;uint32_t sessionId=0;uint32_t sequence=0;uint8_t postureClass=static_cast<uint8_t>(CameraPostureClass::UNKNOWN);uint8_t rawState=0;uint8_t flags=0;uint8_t validFrames=0;uint16_t confidenceMilli=0;float ifScore=0.0f;float ocsvmScore=0.0f;uint32_t inferenceMillis=0;uint16_t checksum=0;};
static_assert(sizeof(CameraStatusPacket)==30);static_assert(sizeof(CameraCommandPacket)==18);static_assert(sizeof(CameraResultPacket)==36);
inline uint16_t cameraStatusChecksum(const CameraStatusPacket&p){return camera_crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(CameraStatusPacket,checksum));}
inline uint16_t cameraCommandChecksum(const CameraCommandPacket&p){return camera_crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(CameraCommandPacket,checksum));}
inline uint16_t cameraResultChecksum(const CameraResultPacket&p){return camera_crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(CameraResultPacket,checksum));}
