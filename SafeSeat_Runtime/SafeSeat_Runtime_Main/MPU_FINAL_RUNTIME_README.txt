SAFESEAT MPU6050 BENCH-RUNTIME INTEGRATION — 2026-08-23
========================================================

Bench/runtime status: PASS WITH OBSERVATION
Actual vehicle/normal-road validation: PENDING

Validated V4 behavior:
- startup installation-angle/gravity/gyro-bias compensation: PASS
- compact standalone cadence: ~80 Hz: PASS
- stationary physical context: STILL: PASS
- sustained gentle movement: vehicle-motion context: PASS
- sustained strong disturbance: STRONG_DISTURBANCE: PASS
- recovery after motion: STILL: PASS
- 3-sample physical-motion debounce: PASS

Fusion semantics now:
1. persistent physical MPU motion is established first;
2. IF/OCSVM then characterize confirmed road/vehicle motion;
3. stationary OCSVM outliers are diagnostic only;
4. ordinary vehicle vibration is supporting context;
5. only persistent strong motion can set the motion-artifact gate;
6. MPU never contributes occupant medical anomaly votes.

Main Runtime sampling:
The standalone V4 validation sketch used 12.5 ms because the lightweight
standalone loop otherwise ran too fast. DO NOT copy that interval into Main.
Main keeps its existing 10 ms acquisition request because the full all-sensor
loop previously yielded about 80–81 Hz actual MPU sampling.

No model retraining, feature changes, scaler changes, IF changes, or OCSVM
parameter changes were made.

Actual vehicle road testing is still required before claiming final
vehicle-domain validation.
