SafeSeat Step 5.9.8.2 - Final Piezo Deployment Policy
======================================================

What changed
------------
Piezo/PVDF is no longer an anomaly-ML node in the deployed runtime.

Remote Piezo node:
  raw ADC @ 25 Hz
  -> EMA smoothing
  -> slow baseline removal
  -> absolute centered-wave excursion detector
  -> cooldown-based breath-event tracking
  -> rolling respiration estimate
  -> conservative no-event support timer
  -> ESP-NOW packet v2

Main Hub:
- consumes deterministic Piezo respiration support
- does NOT expect Piezo IF/OCSVM scores
- Piezo alone can never create WARNING, EMERGENCY, camera trigger, or alert
- a Piezo no-event concern can add one NON-STRONG corroborating anomaly
  vote only when C1001 is already anomalous
- Piezo never adds strongAnomalyEvidenceCount
- C1001 remains the primary vital/respiration source

Important
---------
The event threshold, cooldown, and no-event duration are engineering
signal-processing settings, not diagnostic medical thresholds. They
still require validation on the final installed PVDF seatbelt sleeve.

The WESAD Piezo training artifacts may remain in SafeSeat_ML as an
experimental/historical result, but they are not exported/deployed
into SafeSeat_Runtime_Piezo anymore.
