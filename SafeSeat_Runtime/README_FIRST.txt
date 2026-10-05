SAFESEAT RUNTIME — R5.1 UAT RECORDER + MLX DEMO POLICY
=======================================================
Step: 5.9.9-R5.1
Date: 2026-10-05

WHAT CHANGED
------------
This package is based on the uploaded R5 sensor-value injection runtime.
The researcher console remains at:

  http://192.168.4.1/uat

R5.1 adds:
- Record Session -> downloadable CSV with Fusion/sensor/model evidence.
- MLX Object-Ta < 2 C is diagnostic information only; it no longer makes
  Fusion report TARGET DEGRADED / HELD by itself.
- During active SYNTHETIC MLX UAT injection only, the rapid 1-2 s FOV
  transition hold is bypassed so a controlled jump such as 31 -> 40 C can
  reach the real baseline-relative MLX IF+OCSVM model.
- The live physical MLX path keeps its geometry/FOV guard enabled.

WHAT DID NOT CHANGE
-------------------
- Fusion remains authoritative. The page does not directly force Warning or
  Emergency.
- MLX still uses the 30-s session baseline, 1-s stability check, IF+OCSVM and
  3-block anomaly persistence.
- Existing multi-sensor voting thresholds were not changed.
- Camera remains event-triggered verification only.

FLASHING NOTE
-------------
Flash SafeSeat_Runtime_Main for the R5.1 changes.

R5 already introduced the shared C1001 UAT command. If the C1001 node is not
already running the matching R5 firmware from this package, flash
SafeSeat_Runtime_C1001 as well. Camera firmware is unchanged by R5.1.

READ NEXT
---------
UAT_SENSOR_VALUE_INJECTION_R5_README.txt
UAT_R5_1_CHANGELOG.txt
FINAL_RUNTIME_README.txt
