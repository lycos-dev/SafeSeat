SafeSeat Runtime Main - Step 1 Core
====================================

Purpose:
- One shared I2C initialization in SafeSeat_Runtime_Main.ino
- Standard 100 kHz shared bus during restoration
- Shared ESP32 ADC setup matching the proven combined sketch
- Accurate READY / FAILED startup reporting
- Failed modules are not updated as if they were healthy
- Fusion.cpp and Fusion.h are preserved byte-for-byte
- Sensor module implementations are otherwise unchanged

Replace/copy the entire package into the SafeSeat_Runtime_Main sketch folder.

Expected next step after compile/test:
Step 2 - restore MLX90614 acquisition from the proven combined sketch.
