SafeSeat Step 5.9.8.2 - Piezo Deterministic Respiration Support
================================================================

Deployment change
-----------------
The PVDF/Piezo seatbelt node no longer deploys the WESAD-trained
Isolation Forest / One-Class SVM models.

Final runtime path:
  raw ADC @ 25 Hz
  -> EMA smoothing
  -> slowly-tracking baseline
  -> centered mechanical waveform
  -> conservative event detector
  -> rolling respiration estimate / support timer
  -> ESP-NOW
  -> Main Hub Fusion

Safety / interpretation
-----------------------
- Piezo is secondary mechanical respiration support.
- C1001 remains the primary vital/respiration source.
- Piezo cannot create WARNING, EMERGENCY, camera trigger, or alert by itself.
- The 15-second no-event timer is an engineering corroboration flag,
  not a medical apnea diagnosis.
- The timer is disabled until at least two breath-like events have first
  been observed, reducing startup/disconnected-sensor false concerns.
- Actual thresholds still require validation on the installed PVDF sleeve.

Protocol
--------
PiezoWirePacket protocol version is now 2.
The 40-byte packet carries:
- sensor/signal readiness
- breath tracking readiness
- rolling respiration estimate
- centered waveform sample
- recent/no-breath support state
- breath count, no-event duration, sample count, sample rate

The packet no longer carries IF/OCSVM scores or model flags.
