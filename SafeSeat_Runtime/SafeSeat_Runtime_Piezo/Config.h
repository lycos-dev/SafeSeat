#pragma once

#include <Arduino.h>


// ============================================================
// SAFESEAT PIEZO / SEATBELT ESP32 CONFIG
// ============================================================


// ============================================================
// PIEZO ADC
// ============================================================

constexpr uint8_t PIEZO_PIN = 34;


// ============================================================
// ADC CONFIGURATION
// ============================================================

constexpr uint8_t PIEZO_ADC_RESOLUTION_BITS = 12;


// ============================================================
// MODEL-ALIGNED SAMPLING
//
// WESAD Respiban was preprocessed to 25 Hz.
// Therefore the runtime signal is also sampled at 25 Hz.
//
// 1000 / 25 = 40 ms
// ============================================================

constexpr float PIEZO_SAMPLE_RATE_HZ = 25.0f;

constexpr unsigned long PIEZO_SAMPLE_INTERVAL_MS = 40UL;


// ============================================================
// SIGNAL FILTERING
//
// Preserved from the tested PVDF sketch.
// ============================================================

constexpr float PIEZO_EMA_ALPHA = 0.08f;

constexpr float PIEZO_BASELINE_ALPHA = 0.003f;


// ============================================================
// BREATH DETECTION
//
// These remain auxiliary runtime indicators.
//
// They DO NOT determine the final ML anomaly state.
// ============================================================

constexpr float PIEZO_PEAK_THRESHOLD = 25.0f;

constexpr unsigned long PIEZO_BREATH_COOLDOWN_MS = 1800UL;

constexpr unsigned long PIEZO_APNEA_TIME_MS = 15000UL;


// ============================================================
// TRAINED MODEL WINDOW
//
// 30-second windows at 25 Hz:
//
// 30 × 25 = 750 samples
// ============================================================

constexpr uint16_t PIEZO_WINDOW_SECONDS = 30;

constexpr uint16_t PIEZO_WINDOW_SAMPLES = 750;


// ============================================================
// TRAINED MODEL STRIDE
//
// Training feature engineering used a 5-second stride.
//
// 5 × 25 = 125 samples
// ============================================================

constexpr uint16_t PIEZO_STRIDE_SECONDS = 5;

constexpr uint16_t PIEZO_STRIDE_SAMPLES = 125;


// ============================================================
// SERIAL
// ============================================================

constexpr unsigned long PIEZO_SERIAL_REPORT_INTERVAL_MS = 1000UL;

constexpr bool PIEZO_DEBUG_SERIAL = true;