#pragma once

#include <Arduino.h>

#include "Config.h"

// ============================================================
// SAFESEAT LOCAL WI-FI NETWORK - STEP 5.9.6
//
// The Main Hub is the only SafeSeat device that creates the
// user-facing local Wi-Fi network. The ESP32-S3 camera and phone
// join this AP. C1001 and Piezo remain ESP-NOW sensor nodes.
//
// IMPORTANT:
// Wi-Fi and ESP-NOW share the same 2.4 GHz radio. Keep the AP on
// the same channel used by the SafeSeat ESP-NOW transport.
// Step 5.9.7 will validate all nodes coexisting on this channel.
// ============================================================

constexpr char SAFESEAT_AP_SSID[] = "SafeSeat";
constexpr char SAFESEAT_AP_PASSWORD[] = "safeseat123";

constexpr uint8_t SAFESEAT_AP_CHANNEL = SAFESEAT_ESPNOW_DEFAULT_CHANNEL;
constexpr uint8_t SAFESEAT_AP_MAX_CLIENTS = 4;
constexpr bool SAFESEAT_AP_HIDDEN = false;

// Explicit local address keeps the frontend/API target stable.
// Step 5.9.8 exposes the read-only API at this Main Hub address.
constexpr uint8_t SAFESEAT_AP_IP_A = 192;
constexpr uint8_t SAFESEAT_AP_IP_B = 168;
constexpr uint8_t SAFESEAT_AP_IP_C = 4;
constexpr uint8_t SAFESEAT_AP_IP_D = 1;

constexpr uint8_t SAFESEAT_AP_SUBNET_A = 255;
constexpr uint8_t SAFESEAT_AP_SUBNET_B = 255;
constexpr uint8_t SAFESEAT_AP_SUBNET_C = 255;
constexpr uint8_t SAFESEAT_AP_SUBNET_D = 0;
