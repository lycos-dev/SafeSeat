#pragma once
#include <Arduino.h>

constexpr uint8_t PIEZO_PIN = 34;
constexpr uint8_t PIEZO_ADC_RESOLUTION_BITS = 12;
constexpr int PIEZO_ADC_MIN = 0;
constexpr int PIEZO_ADC_MAX = 4095;

constexpr float PIEZO_SAMPLE_RATE_HZ = 25.0f;
constexpr unsigned long PIEZO_SAMPLE_INTERVAL_MS = 40UL;

// Auxiliary peak-detection waveform only; NOT used as ML source.
constexpr float PIEZO_EMA_ALPHA = 0.08f;
constexpr float PIEZO_BASELINE_ALPHA = 0.003f;

// Training-aligned respiration band.
constexpr float PIEZO_MODEL_LOW_HZ = 0.05f;
constexpr float PIEZO_MODEL_HIGH_HZ = 1.00f;
constexpr uint8_t PIEZO_MODEL_FILTER_ORDER = 4;

// Signal-quality gate. These are engineering guards, not medical thresholds.
constexpr int PIEZO_ADC_RAIL_MARGIN = 8;
constexpr float PIEZO_MAX_RAIL_FRACTION = 0.05f;
constexpr float PIEZO_MIN_ALIGNED_STD = 0.25f;

constexpr float PIEZO_PEAK_THRESHOLD = 25.0f;
constexpr unsigned long PIEZO_BREATH_COOLDOWN_MS = 1800UL;
constexpr unsigned long PIEZO_APNEA_TIME_MS = 15000UL;

constexpr uint16_t PIEZO_WINDOW_SECONDS = 30;
constexpr uint16_t PIEZO_WINDOW_SAMPLES = 750;
constexpr uint16_t PIEZO_STRIDE_SECONDS = 5;
constexpr uint16_t PIEZO_STRIDE_SAMPLES = 125;

constexpr unsigned long PIEZO_SERIAL_REPORT_INTERVAL_MS = 1000UL;
constexpr bool PIEZO_DEBUG_SERIAL = true;

// ============================================================
// PIEZO -> MAIN HUB UART LINK
//
// One-way communication keeps the separate Piezo ESP32
// independent from the Main Hub's C1001 UART.
//
// Wiring:
//   Piezo GPIO17 (TX) -> Main Hub GPIO25 (RX)
//   Piezo GND         -> Main Hub GND
// ============================================================

constexpr uint32_t PIEZO_COMM_BAUD = 115200UL;
constexpr int8_t PIEZO_COMM_TX_PIN = 17;
constexpr unsigned long PIEZO_COMM_TX_INTERVAL_MS = 500UL;
