#pragma once
#include <Arduino.h>

// ============================================================
// SAFESEAT PIEZO / PVDF SEATBELT NODE
// DETERMINISTIC RESPIRATION SUPPORT - STEP 5.9.8.2
//
// The Piezo node no longer runs Isolation Forest / One-Class SVM.
// It provides conservative mechanical-respiration support only.
//
// IMPORTANT:
// - These are engineering signal-processing parameters.
// - They are NOT medical diagnostic thresholds.
// - Piezo evidence can support C1001/Fusion but cannot create
//   WARNING/EMERGENCY by itself.
// ============================================================

constexpr uint8_t PIEZO_PIN = 34;
constexpr uint8_t PIEZO_ADC_RESOLUTION_BITS = 12;
constexpr int PIEZO_ADC_MIN = 0;
constexpr int PIEZO_ADC_MAX = 4095;

// 25 Hz is retained from the already-integrated Piezo runtime.
constexpr float PIEZO_SAMPLE_RATE_HZ = 25.0f;
constexpr unsigned long PIEZO_SAMPLE_INTERVAL_MS = 40UL;

// Two-stage low-complexity signal path:
// raw ADC -> EMA -> slowly-tracking baseline -> centered waveform.
constexpr float PIEZO_EMA_ALPHA = 0.08f;
constexpr float PIEZO_BASELINE_ALPHA = 0.003f;

// Event detector parameters. The absolute centered waveform is used
// so the sensor's mounting polarity does not matter. Cooldown helps
// prevent the positive/negative lobes of one mechanical breath cycle
// from being double-counted.
constexpr float PIEZO_EVENT_THRESHOLD = 25.0f;
constexpr unsigned long PIEZO_BREATH_COOLDOWN_MS = 1800UL;

// Do not evaluate the no-breath timer until the node has first seen
// enough repeatable breath events to establish that the belt/PVDF
// coupling is actually working.
constexpr uint8_t PIEZO_MIN_BREATHS_FOR_TRACKING = 2;

// Provisional engineering support timer only. It is never a standalone
// medical decision and never creates WARNING/EMERGENCY on its own.
constexpr unsigned long PIEZO_NO_BREATH_SUPPORT_MS = 15000UL;

// Small startup settle period for EMA/baseline initialization.
constexpr unsigned long PIEZO_STARTUP_SETTLE_MS = 3000UL;

constexpr unsigned long PIEZO_SERIAL_REPORT_INTERVAL_MS = 1000UL;
constexpr bool PIEZO_DEBUG_SERIAL = true;

// ============================================================
// PIEZO -> MAIN HUB ESP-NOW LINK
// ============================================================

constexpr uint8_t SAFESEAT_ESPNOW_DEFAULT_CHANNEL = 6;
constexpr uint8_t SAFESEAT_ESPNOW_MIN_CHANNEL = 1;
constexpr uint8_t SAFESEAT_ESPNOW_MAX_CHANNEL = 13;
constexpr unsigned long SAFESEAT_ESPNOW_CHANNEL_DWELL_MS = 350UL;
constexpr unsigned long SAFESEAT_ESPNOW_HUB_BEACON_TIMEOUT_MS = 2000UL;
constexpr unsigned long PIEZO_COMM_TX_INTERVAL_MS = 500UL;
