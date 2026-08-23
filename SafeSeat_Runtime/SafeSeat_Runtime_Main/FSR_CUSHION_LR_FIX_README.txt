SAFESEAT FSR CUSHION LEFT/RIGHT MAPPING FIX — 2026-08-23
========================================================

Live combined-hardware testing showed the cushion left/right wiring is mirrored
relative to the previous software assumption.

Coordinate convention:
  LEFT / RIGHT are from the seated occupant's perspective.

Correct physical mapping:
  Physical LEFT   = GPIO34 = electrical cushion FSR3
  Physical CENTER = ADS1115 #2 A3 = electrical cushion FSR2
  Physical RIGHT  = ADS1115 #2 A2 = electrical cushion FSR1

Only the logical mapping was corrected. No FSR ML model, feature order, model
parameters, or hardware wiring was changed.

The logical/model order remains:
  CushionLeft, CushionCenter, CushionRight

Therefore downstream FSR ML receives the intended physical L/C/R order after
this mapping correction.
