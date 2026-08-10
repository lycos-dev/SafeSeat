#pragma once

#include <Arduino.h>

// ============================================================
// SAFESEAT REMOTE C1001 NODE - STEP 5.8
// ============================================================

// C1001 UART wiring on the dedicated ESP32.
constexpr uint8_t C1001_RX_PIN = 16;
constexpr uint8_t C1001_TX_PIN = 17;

// Serial dashboard.
constexpr unsigned long C1001_NODE_PRINT_INTERVAL_MS = 1000UL;

// ESP-NOW channel discovery. The Main Hub broadcasts a beacon
// on its current Wi-Fi channel; this node scans until it hears it.
constexpr uint8_t SAFESEAT_ESPNOW_DEFAULT_CHANNEL = 6;
constexpr uint8_t SAFESEAT_ESPNOW_MIN_CHANNEL = 1;
constexpr uint8_t SAFESEAT_ESPNOW_MAX_CHANNEL = 13;
constexpr unsigned long SAFESEAT_ESPNOW_CHANNEL_DWELL_MS = 180UL;
constexpr unsigned long SAFESEAT_ESPNOW_HUB_BEACON_TIMEOUT_MS = 2500UL;

// Evidence packet cadence. C1001 itself samples at 1 Hz; the
// link refreshes twice per second so stale-link detection is fast.
constexpr unsigned long C1001_COMM_TX_INTERVAL_MS = 500UL;
