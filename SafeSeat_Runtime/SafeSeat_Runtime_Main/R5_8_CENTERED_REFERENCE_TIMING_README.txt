SafeSeat Runtime Main R5.8 - Centered FSR Reference + Practical Persistence
=============================================================================
Base: R5.7 Adaptive Validation Final

Why this revision exists
------------------------
R5.7 correctly made FSR posture passenger-relative, but vehicle testing exposed an
important real-world entry case: a passenger can enter the vehicle leaning toward
the door/one side. If that temporary side-entry posture is learned as the normal
reference, becoming upright later can look like a false lean in the opposite
direction.

R5.8 changes only the reference/timing behavior needed to address that issue while
keeping the R5.7 validation, MLX hot-water, Monitoring Limited, camera, and severity
architecture intact.

FSR reference acquisition
-------------------------
- Occupant pressure first enters a 10-second ENTRY GRACE period.
- During ENTRY GRACE, no left/right posture is learned as the passenger reference.
- After entry grace, the backrest must be reasonably CENTERED (backrest LR balance
  within +/-0.20), with usable cushion/backrest contact.
- Centered posture must remain continuous for about 6 seconds before the reference
  locks.
- If the passenger is still leaning to one side, reference learning waits instead
  of learning that side posture as normal.
- If the passenger later becomes upright/centered, the 6-second centered-reference
  timer starts then.
- Once learned, the reference slowly adapts only during clearly centered normal
  samples. It never adapts toward an active side/forward posture candidate.

Left/right posture classification
---------------------------------
A side posture now needs BOTH:
1. a meaningful shift from the passenger's centered reference; and
2. the current pressure distribution to be physically biased to that same side.

This prevents the R5.7 failure mode where a right-heavy reference could make an
upright posture look LEFT merely because it moved left relative to the bad reference.

FSR posture timing
------------------
- Sustained LEFT / RIGHT / FORWARD posture: 12 seconds before strong FSR vote.
- FSR strong posture evidence alone remains WATCH-only.
- Brief reaching, belt adjustment, bag movement, road repositioning, and normal
  short posture changes are expected to reset before the 12-second threshold.
- Forward posture remains separate from side posture escalation:
  * >=30 seconds: Monitoring Limited notice.
  * >=60 seconds: persistent Monitoring Limited.
  * Monitoring Limited never creates WARNING or EMERGENCY by itself.

MLX / C1001 persistence
-----------------------
- Trusted STRONG MLX: 10 seconds continuous critical persistence -> WARNING.
- Trusted STRONG C1001: 10 seconds continuous critical persistence -> WARNING,
  provided the C1001 motion-artifact gate says the reading is reliable.
- The 10-second interval is a system false-positive rejection/persistence threshold,
  not a medical diagnostic threshold.
- After WARNING is confirmed, the qualifying critical condition must remain active
  for another 30 continuous seconds -> EMERGENCY.
- Clearing the critical condition resets the Emergency timer.

Unchanged from R5.7
-------------------
- /validation auto-starts Validation on LIVE READINGS.
- Live Readings clears test-input overrides without leaving Validation.
- Completed MLX baseline/model state is preserved when returning to live readings.
- Physical MLX hot-water transitions use the validation transition profile.
- Strong trusted MLX or C1001 can independently escalate.
- FSR alone cannot create WARNING/EMERGENCY.
- Camera remains corroboration only.
- MPU remains motion/context only.
- All 9 FSR values remain visible.
- Forward posture Monitoring Limited remains non-emergency information.

Validation console diagnostics
------------------------------
The FSR status line now distinguishes:
- ENTRY SETTLING x / 10 s
- WAITING FOR CENTERED POSTURE
- CENTERING x / 6 s
- REFERENCE READY
- LEFT / RIGHT / FORWARD x / 12 s

The recorder now also exports entry timing, centered status, current LR balance,
and learned reference LR balance so future physical tests can be diagnosed directly.

Observed-session replay check
-----------------------------
The 2026-10-12 vehicle validation CSV that exposed the issue was replayed against the
R5.8 centered-reference rules. The original side-entry segment (about -0.37 LR
balance) is rejected as a reference. The reference locks only after the later
centered segment, at approximately 39.4 s in the 1 Hz exported recording, with a
reference LR balance near -0.006. The later centered values are no longer classified
as LEFT by the R5.8 side-direction rules.

Build note
----------
Host-side source/static checks and behavioral replay checks are included in
R5_8_CENTERED_REFERENCE_TIMING_CHECKS.json. A final ESP32 board Verify/Compile should
still be run in the user's Arduino/ESP32 environment before flashing.
