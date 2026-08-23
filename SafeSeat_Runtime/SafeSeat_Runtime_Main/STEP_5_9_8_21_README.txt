SAFESEAT STEP 5.9.8.21 — TINY ML STATUS HEARTBEAT
2026-08-23
==================================================

BASE
----
Built directly from the user's working Step 5.9.8.20 runtime.

FUNCTIONAL SENSOR/MODEL LOGIC
-----------------------------
UNCHANGED.

Only SafeSeat_Runtime_Main.ino display text was changed:
- version label
- corrected "1-second heartbeat" boot wording
- compact FSR/MPU/MLX model status labels
- compact Fusion level

MLX BASELINE BEHAVIOR
---------------------
This is intentional:

A) Before the personal baseline reaches 30/30:
   If the nape/FOV becomes invalid, the partial baseline is reset.
   Reason: do not keep a partially contaminated baseline.

B) After the personal baseline reaches 30/30:
   Temporary lean-forward/FOV loss does NOT reset the baseline.
   MLXML is held and excluded from Fusion.
   Heartbeat shows:
      MLX=HELD ... base=30/30 ... MLXML=HOLD

C) Actual seat exit:
   The thermal session ends and baseline resets to 0/30.
   Heartbeat shows:
      occ=NO ... MLX=IDLE base=0/30 MLXML=IDLE

TINY STATUS LABELS
------------------
FSRML:
  WAIT / NORMAL / WEAK / STRONG

MPUL:
  OFF / CAL / WAIT / NORMAL / ROAD+ / ROAD++

MLXML:
  IDLE / NO-TARGET / BASE / HOLD / WAIT / NORMAL / WEAK / STRONG

FUS:
  SAFE / WATCH / WARNING / EMERGENCY (according to Fusion)

SERIAL
------
Upload Speed   : 115200 recommended
Serial Monitor : 460800

The verbose dashboard remains disabled.
