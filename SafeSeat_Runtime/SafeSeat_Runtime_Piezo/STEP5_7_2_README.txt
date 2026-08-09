SafeSeat Step 5.7.2 - Piezo Embedded ML + Main Hub Communication
======================================================================

MODEL CONTRACT
--------------
Source: WESAD RespiBAN surrogate respiratory-motion data
Runtime rate: 25 Hz
Window: 750 samples / 30 seconds
Stride: 125 samples / 5 seconds
Features: 16
Scaler: StandardScaler
Isolation Forest: 300 trees, max_samples=256
One-Class SVM: RBF, gamma=0.01, 34 support vectors

Step 5.7 training and runtime now use the same order:
  raw 25 Hz window
  -> linear detrend
  -> 0.05-1.0 Hz SOS forward/reverse
  -> per-window median/MAD normalization
  -> 16 features
  -> StandardScaler
  -> IF + OCSVM

The remaining limitation is sensor-domain transfer:
WESAD inductive RespiBAN != SafeSeat PVDF seatbelt sensor.

IMPORTANT MODEL FIX
-------------------
The fitted StandardScaler has a near-zero-but-nonzero scale for the
median feature (~8e-18). The runtime must preserve it. Step 5.7.2 only
falls back to scale=1 for exactly-zero or non-finite scales.

Validation against all 1,717 held-out feature windows:
  IF predictions:    1717/1717 match
  OCSVM predictions: 1717/1717 match

COMMUNICATION
-------------
One-way UART:
  Piezo ESP32 GPIO17 (TX) -> Main Hub GPIO25 (RX)
  Piezo ESP32 GND         -> Main Hub GND

Baud: 115200
Packet: 40 bytes + CRC16
Send interval: 500 ms

Before the first valid 30-second window, the Main Hub will see the
Piezo link but model evidence will remain not ready.

FUSION SAFETY POLICY
--------------------
Piezo is corroborating respiration-pattern evidence.
It is never allowed to create a standalone emergency.

Do not interpret IF/OCSVM outputs as a medical diagnosis.
The auxiliary peak/no-breath logic remains an engineering context signal.
