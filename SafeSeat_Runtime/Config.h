#pragma once

#include <Arduino.h>

// ============================================================
// SAFESEAT MAIN ESP32 CONFIGURATION
// ============================================================

// -------------------- I2C --------------------
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

// -------------------- C1001 UART --------------------
constexpr uint8_t C1001_RX_PIN = 16;
constexpr uint8_t C1001_TX_PIN = 17;
constexpr uint32_t C1001_BAUD = 115200;

// -------------------- C1001 warm-up --------------------
constexpr unsigned long C1001_WARMUP_MS = 60000UL;

// -------------------- ADS1115 --------------------
constexpr uint8_t ADS1115_1_ADDRESS = 0x48;
constexpr uint8_t ADS1115_2_ADDRESS = 0x49;

// -------------------- FSR layout --------------------
//
// ADS1115 #1
// A0 = Backrest FSR1
// A1 = Backrest FSR2
// A2 = Backrest FSR3
// A3 = Backrest FSR4
//
// ADS1115 #2
// A0 = Backrest FSR5
// A1 = Backrest FSR6
// A2 = Cushion FSR1
// A3 = Cushion FSR2
//
// Native ADC
// GPIO34 = Cushion FSR3
//
constexpr uint8_t CUSHION_FSR3_PIN = 34;

// -------------------- MPU6050 --------------------
constexpr uint8_t MPU6050_ADDRESS = 0x68;

// -------------------- Sampling --------------------

// Main acquisition loop.
// 50 ms = 20 Hz.
//
// We can later decimate/resample features as needed per model.
constexpr unsigned long SENSOR_SAMPLE_INTERVAL_MS = 50UL;

// Serial diagnostics interval.
constexpr unsigned long SERIAL_REPORT_INTERVAL_MS = 500UL;

// -------------------- Filtering --------------------

// General EMA coefficients.
// These are runtime signal-conditioning defaults,
// not ML decision thresholds.
constexpr float MLX_EMA_ALPHA = 0.35f;

constexpr float FSR_EMA_ALPHA = 0.35f;

constexpr float MPU_EMA_ALPHA = 0.30f;

// -------------------- FSR --------------------

// ADS1115 gain setting will be configured in FSR.cpp.
//
// Keep raw/filtered ADC values available.
// Calibration to physical/live scale is handled separately.

constexpr int FSR_MEDIAN_WINDOW = 5;

// -------------------- Runtime --------------------

constexpr bool ENABLE_DEBUG_SERIAL = true;