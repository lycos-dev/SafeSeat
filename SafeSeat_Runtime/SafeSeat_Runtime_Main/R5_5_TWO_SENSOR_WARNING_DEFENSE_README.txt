SafeSeat Main Hub R5.5 - Two-Sensor Warning / Final Defense Revision

Base: R5.4 Final Defense Fusion/UAT.

1. WARNING now requires agreement from at least TWO independent STRONG anomaly
   modalities among C1001, FSR and MLX90614. A single strong sensor, any weak
   anomaly, or an occupancy conflict is WATCH only. FSR alone can therefore no
   longer raise WARNING.

2. The controlled physical/injected FSR hard-side pattern still requires 10 s
   continuous persistence before it contributes its one strong FSR vote. During
   the defense flow, pair that vote with a real MLX strong anomaly to demonstrate
   WARNING. C1001 remains a valid third decision modality when its real remote
   IF+OCSVM window is anomalous.

3. Camera remains event-triggered CORROBORATION ONLY; MPU remains motion/artifact
   context only. Neither can satisfy the two-sensor WARNING requirement.

4. If two strong sensor votes remain continuously present, the existing strong
   multisensor timer continues. At 30,000 ms the Fusion state becomes EMERGENCY.
   Breaking the two-strong-sensor agreement resets the 30 s timer immediately.

5. De-escalation was aligned with the new policy. If WARNING/EMERGENCY loses its
   two-sensor agreement, the elevated state is held for the existing 3 s recovery
   hysteresis. If one anomaly remains after that, Fusion becomes WATCH rather than
   staying latched in WARNING; when concerns clear and readiness is sufficient it
   can return to SAFE.

6. /uat now clearly shows WARNING agreement (x/2 strong), Emergency escalation
   (elapsed/30 s), and the Camera Registered Control Mode. Hidden app taps are
   therefore visible on /uat as REAL CAMERA, CONTROLLED UPRIGHT, or CONTROLLED
   NON-UPRIGHT while Controlled UAT is active.

7. Existing ML baselines/windows, camera event-triggering, MLX rapid-transition
   UAT bypass, CSV recording, and all sensor model implementations are preserved.

Only the complete SafeSeat_Runtime_Main folder is packaged. Dedicated C1001 and
ESP32-S3 camera firmware are not changed by R5.5.
