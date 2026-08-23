SAFESEAT RUNTIME — ARDUINO-ESP32 3.3.10 COMPILATION FIX
2026-08-23
==========================================================

The previous stall-guard build used the Arduino-ESP32 2.x watchdog API:

  esp_task_wdt_init(seconds, panic)

Arduino-ESP32 3.3.10 uses ESP-IDF 5.x, where the API is:

  esp_task_wdt_init(const esp_task_wdt_config_t *config)

This package supports BOTH API generations using ESP_IDF_VERSION_MAJOR.

The live heartbeat also previously referenced non-existent fields:
  m.actualFs
  f.actualFs

The actual runtime structs use:
  m.actualSamplingRateHz
  f.actualSamplingRateHz

Those references are corrected.

RUNTIME SERIAL SETTINGS FOR THIS PACKAGE
----------------------------------------
Tools > Upload Speed : 115200 recommended
Serial Monitor       : 460800

Sensor/model logic is unchanged from the parent package.
