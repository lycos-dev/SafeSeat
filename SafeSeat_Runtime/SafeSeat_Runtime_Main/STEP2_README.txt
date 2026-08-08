SafeSeat Runtime Main - Step 2 MLX Restoration
================================================

Changed:
- MLX.h
- MLX.cpp

Preserved:
- Step 1 shared hardware foundation
- C1001 module
- FSR module
- MPU module
- Fusion.cpp and Fusion.h remain untouched

MLX behavior restored from the proven combined sketch:
1. Existing shared Wire bus
2. mlx.begin()
3. readAmbientTempC()
4. readObjectTempC()
5. sensor-specification validity check
6. 3-sample median
7. EMA alpha = 0.35
8. object-minus-ambient context

Important improvement:
An isolated invalid current MLX sample no longer destroys availability
of the previously trusted filtered value. Status reports that the old
value is being held while currentSampleAccepted=false.

Next:
Compile/upload and check MLX before Step 3 FSR restoration.
