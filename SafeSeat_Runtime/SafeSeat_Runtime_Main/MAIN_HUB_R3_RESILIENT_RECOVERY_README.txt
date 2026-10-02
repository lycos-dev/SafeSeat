SafeSeat Main Hub 5.9.9-R3
Resilient I2C / FSR recovery + app-local UAT simulation

Changes from R2:
1. Retained FSR empty-seat baseline may now be restored after:
   - SafeSeat supervisor software restart
   - panic reset
   - interrupt/task/other watchdog reset
   It is still NOT restored after power-on, brownout, external/manual reset,
   or deep-sleep wake. Those paths keep normal empty-seat calibration.

2. Shared I2C transactions use a bounded timeout.

3. FSR recovery is staged:
   - one-off ADS probe fault -> reinitialize only the affected ADS device
   - repeated ADS fault -> clear/restart shared SDA/SCL bus, then reinitialize both ADS1115s
   - repeated acquisition fault -> shared-bus recovery
   - sustained unexpected all-zero ADS frames -> shared-bus recovery

4. Shared-bus recovery performs up to 9 SCL pulses if SDA is held low,
   generates a STOP condition, restarts Wire at the validated 100 kHz, and
   reconnects both ADS1115 devices without erasing the retained baseline.

5. One transient FSR read failure no longer immediately becomes a permanent-
   looking DEGRADED state. Short failures enter RECOVERING and are escalated
   only after a small consecutive-failure streak.

6. The old auto-refreshing /uat browser evaluator is disabled. /uat is now a
   lightweight page only. Participant UAT state simulation is local to the
   SafeSeat app under Settings > Diagnostics and sends no test command to the
   Main Hub. Serial engineering commands are retained.

Important:
- This improves software recovery from a wedged I2C transaction/peripheral.
- It cannot repair a physically disconnected ADS1115, bad power rail, loose
  SDA/SCL/GND wiring, or a true brownout. Reset reason is still printed at boot
  so those cases can be distinguished.
