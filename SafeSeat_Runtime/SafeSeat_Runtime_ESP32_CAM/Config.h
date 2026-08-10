#pragma once

#include <Arduino.h>

// ============================================================
// SAFESEAT ESP32-CAM CONFIGURATION - STEP 5.9.4
// Target: AI-Thinker ESP32-CAM + OV2640
// Programming/power assumption: FTDI, ESP32-CAM 5V input.
// ============================================================

constexpr uint8_t SAFESEAT_ESPNOW_DEFAULT_CHANNEL = 6;
constexpr uint8_t SAFESEAT_ESPNOW_MIN_CHANNEL = 1;
constexpr uint8_t SAFESEAT_ESPNOW_MAX_CHANNEL = 13;
constexpr unsigned long SAFESEAT_ESPNOW_BEACON_STALE_MS = 1500UL;
constexpr unsigned long SAFESEAT_ESPNOW_SCAN_DWELL_MS = 180UL;
constexpr unsigned long CAMERA_STATUS_INTERVAL_MS = 1000UL;

constexpr uint16_t CAMERA_VERIFY_FRAME_COUNT = 3;
constexpr uint16_t CAMERA_VERIFY_MIN_VALID_FRAMES = 2;
constexpr unsigned long CAMERA_INTER_FRAME_DELAY_MS = 150UL;
constexpr uint8_t CAMERA_WARMUP_DISCARD_FRAMES = 2;

// Tensor arena is allocated from PSRAM. The model itself remains
// in flash as g_posture_model[] (~400 KiB).
constexpr size_t CAMERA_TENSOR_ARENA_BYTES = 2u * 1024u * 1024u;

constexpr int CAMERA_MODEL_INPUT_WIDTH = 160;
constexpr int CAMERA_MODEL_INPUT_HEIGHT = 160;
constexpr int CAMERA_MODEL_INPUT_CHANNELS = 3;
constexpr int CAMERA_MODEL_CLASS_COUNT = 5;

// The source camera frame is QQVGA JPEG (160x120). It is decoded
// to RGB888 and stretched to 160x160, matching the training-time
// square resize behavior.
constexpr int CAMERA_SOURCE_WIDTH = 160;
constexpr int CAMERA_SOURCE_HEIGHT = 120;

// Default physical orientation. Keep horizontal mirroring OFF
// because left/right are distinct model classes.
constexpr bool CAMERA_VERTICAL_FLIP = false;
constexpr bool CAMERA_HORIZONTAL_MIRROR = false;
