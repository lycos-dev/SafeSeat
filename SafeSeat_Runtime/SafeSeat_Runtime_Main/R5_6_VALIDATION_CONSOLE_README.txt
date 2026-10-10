SafeSeat Main Hub R5.6 - System Validation Console
=================================================

Base: R5.5 Two-Sensor Final Defense

What changed
------------
1. Added /validation as the recommended defense/engineering console URL.
   /uat remains as a legacy alias so existing tools and app integrations are not broken.

2. Rebuilt the browser console as "SafeSeat System Validation Console".
   Visible wording no longer uses Controlled, Inject, or UAT terminology.
   Optional non-live values are clearly called Sensor Test Inputs / Test Conditions.

3. Preserved the live sensor detail that was already useful, including all 9 FSR values.
   The FSR array is now presented more clearly without removing any of the nine channels.

4. Added System Readiness for the FSR + MLX physical demonstration.
   Readiness uses real live telemetry and real model/baseline state; no sample counts are hard-coded.

5. Added explicit effective Fusion vote telemetry.
   The API now exposes which of C1001, FSR and MLX are actually providing STRONG or WEAK anomaly votes.
   This does not change the R5.5 2-of-3 Fusion policy.

6. Added Decision Monitor and a browser-side event timeline.
   It shows the current effective sensor votes, 2-sensor agreement, and the real 30-second Emergency persistence timer.

7. Camera verification source is shown as Live Camera, Upright Reference, or Non-Upright Reference.
   The hidden app control still uses the existing API and remains compatible.
   Camera remains corroboration-only and never changes Fusion severity.

8. Added public validation API aliases under /api/v1/validation/input/*.
   Existing /api/v1/uat/injection/* routes remain for backward compatibility.

9. Added system.external_escalation_allowed telemetry.
   It is false while a validation session is active and true during normal live operation.
   IMPORTANT: this is the Runtime-side safety signal only. The current mobile app must explicitly consume
   this field before it can be relied on to suppress cloud/SMS escalation during a validation Emergency.

What did NOT change
-------------------
- One strong sensor remains WATCH.
- Two independent strong sensors are required for WARNING.
- Valid decision modalities remain C1001, FSR and MLX.
- Camera and MPU do not count toward Warning/Emergency agreement.
- The physical validation FSR side-load persistence remains about 10 seconds.
- Emergency requires 30 continuous seconds of two-sensor strong agreement.
- Breaking the two-sensor pair resets the Emergency timer.
- Existing baseline/model windows remain preserved when validation begins/updates.
- Existing sensor models and thresholds are unchanged.

Recommended defense URL
-----------------------
http://192.168.4.1/validation
