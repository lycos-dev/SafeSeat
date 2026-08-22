#pragma once

#include <Arduino.h>


// ============================================================
// SAFESEAT MAIN HUB CONFIGURATION
//
// STEP 1:
// Shared hardware initialization only.
//
// Sensor acquisition/filter implementations are intentionally
// left unchanged until their individual restoration steps.
// ============================================================


// ============================================================
// SHARED I2C BUS
//
// Proven combined sketch uses:
//
//     Wire.begin(21, 22);
//
// The MLX90614 shares this bus with the two ADS1115 boards and
// MPU6050. Keep the shared bus at standard 100 kHz for the
// acquisition-restoration stage.
//
// IMPORTANT:
// The previous Runtime_Main forced 400 kHz globally. We are
// removing that here because the proven combined sketch did
// not do so, and the MLX90614/SMBus path should not be forced
// to fast-mode during this restoration stage.
// ============================================================

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

constexpr uint32_t I2C_CLOCK_HZ = 100000UL;


// ============================================================
// SHARED ESP32 ADC CONFIGURATION
//
// Proven combined sketch used:
//     analogReadResolution(12);
//     analogSetAttenuation(ADC_11db);
//
// This applies to the native ADC input used for Cushion FSR3.
// ============================================================

constexpr uint8_t MAIN_ADC_RESOLUTION_BITS = 12;


// ============================================================
// REMOTE C1001
//
// Step 5.8: C1001 is no longer wired to this Main Hub.
// Acquisition + filtering + ML run on a dedicated ESP32 node.
// Main receives its evidence over ESP-NOW.
// ============================================================

// ============================================================
// ADS1115
// ============================================================

constexpr uint8_t ADS1115_1_ADDRESS = 0x48;
constexpr uint8_t ADS1115_2_ADDRESS = 0x49;


// ============================================================
// FSR NATIVE ADC
//
// Cushion FSR3
// ============================================================

constexpr uint8_t CUSHION_FSR3_PIN = 34;


// ============================================================
// MPU6050
// ============================================================

constexpr uint8_t MPU6050_ADDRESS = 0x68;


// ============================================================
// SERIAL DASHBOARD
// ============================================================

constexpr unsigned long MAIN_PRINT_INTERVAL_MS = 1000UL;


// ============================================================
// DEBUG
// ============================================================

constexpr bool ENABLE_DEBUG_SERIAL = true;

// ============================================================
// SAFESEAT ESP-NOW REMOTE LINK
//
// Main Hub broadcasts a small channel beacon and receives remote
// C1001 evidence plus ESP32-S3 camera status/results wirelessly.
//
// Channel 6 is the SafeSeat local-network channel. Remote nodes
// follow the Main Hub beacon and remain on that channel.
// ============================================================

constexpr uint8_t SAFESEAT_ESPNOW_DEFAULT_CHANNEL = 6;
constexpr unsigned long SAFESEAT_ESPNOW_HUB_BEACON_INTERVAL_MS = 250UL;
constexpr unsigned long C1001_COMM_FRESHNESS_TIMEOUT_MS = 2500UL;

// ============================================================
// ESP32-CAM VERIFICATION LINK - STEP 5.9.4
// ============================================================

constexpr unsigned long CAMERA_COMM_FRESHNESS_TIMEOUT_MS = 3000UL;
constexpr unsigned long CAMERA_RESULT_FRESHNESS_TIMEOUT_MS = 5000UL;
constexpr unsigned long CAMERA_TRIGGER_RETRY_MS = 500UL;
constexpr unsigned long CAMERA_REQUEST_TIMEOUT_MS = 12000UL;
constexpr uint8_t CAMERA_VERIFY_FRAME_COUNT = 3;
constexpr uint8_t CAMERA_VERIFY_MIN_VALID_FRAMES = 2;
