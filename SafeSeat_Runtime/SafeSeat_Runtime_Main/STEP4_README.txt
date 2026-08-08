SafeSeat Runtime Main - Step 4 MPU6050 Restoration
===================================================

Changed:
- MPU.h
- MPU.cpp

Preserved byte-for-byte:
- SafeSeat_Runtime_Main.ino
- Config.h
- C1001 module
- MLX Step 2 module
- FSR Step 3 module
- Fusion.cpp / Fusion.h

Restored from the proven combined sketch:
- raw I2C register write
- wake: 0x6B <- 0x00
- gyro config: 0x1B <- 0x00
- accel config: 0x1C <- 0x00
- 14-byte read from register 0x3B
- +/-2g conversion: 16384 LSB/g
- +/-250 dps conversion: 131 LSB/(deg/s)
- temperature conversion: raw/340 + 36.53

Important:
The previous modular code used a separate connection-test abstraction.
Step 4 instead verifies the exact 14-byte read operation that the
working combined sketch relied on.

Retained on top of proven acquisition:
- accel magnitude
- gyro magnitude
- dynamic acceleration
- ~100 Hz runtime timing diagnostics

Next:
Compile/upload. If C1001 + MLX + FSR + MPU are now all valid together,
the acquisition-restoration phase is complete and Fusion can be built.
