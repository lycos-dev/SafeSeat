#pragma once

#include <Arduino.h>

// ============================================================
// SAFESEAT ESP32-S3 WROOM CAMERA CONFIGURATION - STEP 5.9.5
// ============================================================

// ---------- SafeSeat Wi-Fi station foundation ----------
// Step 5.9.6 will make the Main Hub create this SoftAP.
// Until then, this camera node retries periodically and falls back
// to ESP-NOW channel discovery between Wi-Fi association attempts.
constexpr bool SAFESEAT_WIFI_STA_ENABLED = true;
constexpr char SAFESEAT_WIFI_SSID[] = "SafeSeat";
constexpr char SAFESEAT_WIFI_PASSWORD[] = "safeseat123";
constexpr unsigned long SAFESEAT_WIFI_CONNECT_TIMEOUT_MS = 5000UL;
constexpr unsigned long SAFESEAT_WIFI_RETRY_INTERVAL_MS = 30000UL;

// ---------- ESP-NOW ----------
constexpr uint8_t SAFESEAT_ESPNOW_DEFAULT_CHANNEL = 6;
constexpr uint8_t SAFESEAT_ESPNOW_MIN_CHANNEL = 1;
constexpr uint8_t SAFESEAT_ESPNOW_MAX_CHANNEL = 13;
constexpr unsigned long SAFESEAT_ESPNOW_BEACON_STALE_MS = 1500UL;
constexpr unsigned long SAFESEAT_ESPNOW_SCAN_DWELL_MS = 180UL;
constexpr unsigned long CAMERA_STATUS_INTERVAL_MS = 1000UL;

// ---------- Verification cycle ----------
constexpr uint16_t CAMERA_VERIFY_FRAME_COUNT = 3;
constexpr uint16_t CAMERA_VERIFY_MIN_VALID_FRAMES = 2;
constexpr unsigned long CAMERA_INTER_FRAME_DELAY_MS = 150UL;
constexpr uint8_t CAMERA_WARMUP_DISCARD_FRAMES = 2;

// ---------- Model ----------
constexpr size_t CAMERA_TENSOR_ARENA_BYTES = 2u * 1024u * 1024u;
constexpr int CAMERA_MODEL_INPUT_WIDTH = 160;
constexpr int CAMERA_MODEL_INPUT_HEIGHT = 160;
constexpr int CAMERA_MODEL_INPUT_CHANNELS = 3;
constexpr int CAMERA_MODEL_CLASS_COUNT = 5;

// The production capture is QQVGA JPEG (160x120).  The RGB buffer
// is deliberately sized up to QVGA (320x240) as a safety margin so
// a future diagnostic frame-size change cannot overrun the buffer.
constexpr int CAMERA_CAPTURE_WIDTH = 160;
constexpr int CAMERA_CAPTURE_HEIGHT = 120;
constexpr int CAMERA_RGB_BUFFER_MAX_WIDTH = 320;
constexpr int CAMERA_RGB_BUFFER_MAX_HEIGHT = 240;
constexpr size_t CAMERA_RGB_BUFFER_BYTES =
    static_cast<size_t>(CAMERA_RGB_BUFFER_MAX_WIDTH)
    * CAMERA_RGB_BUFFER_MAX_HEIGHT * 3u;

// Keep horizontal mirroring OFF: left/right are distinct classes.
constexpr bool CAMERA_VERTICAL_FLIP = false;
constexpr bool CAMERA_HORIZONTAL_MIRROR = false;

// This ESP32-S3 camera wiring has PWDN=-1.  Therefore the final
// event-driven policy is: camera driver stays initialized, but frames
// and CNN inference are performed only when Main requests verification.
constexpr bool CAMERA_EVENT_CAPTURE_ONLY = true;
