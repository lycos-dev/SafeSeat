SafeSeat Runtime - Step 5.9.8.2
===================================

This package updates the final deployed Piezo/PVDF path from the earlier
surrogate-trained IF/OCSVM design to deterministic respiration support.

Changed:
- SafeSeat_Runtime_Piezo
- SafeSeat_Runtime_Main Piezo protocol/communication/Fusion/API/dashboard

Preserved:
- C1001 remote ML/runtime
- MLX context/runtime
- FSR ML/runtime
- MPU ML/context runtime
- ESP32-S3 camera INT8 verifier
- SafeSeat SoftAP
- local read-only telemetry API
- camera transaction logic

Piezo final role:
PVDF raw ADC -> EMA -> slow baseline -> mechanical breath events
-> rolling respiration estimate / conservative no-event support
-> ESP-NOW -> Main Fusion.

The Piezo node cannot independently create WARNING, EMERGENCY, camera trigger,
or alert. C1001 remains the primary vital/respiration source.

The WESAD Piezo model-development artifacts in SafeSeat_ML are retained only as
experimental/historical work and are not deployed into this runtime.
