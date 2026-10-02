SafeSeat Main Hub 5.9.9-R4 — Maintenance /uat restoration

Changes from R3:
- Restores /uat as a read-only technical maintenance monitor.
- Adds /maintenance alias to the same page.
- Page uses only GET /api/v1/status (one HTTP request per refresh).
- Manual refresh is the default.
- Optional Live Maintenance refreshes once every 5 seconds.
- Hub-side Warning/Emergency simulation remains disabled.
- App researcher long-press simulation remains the UAT stimulus path.
- R3 freeze recovery, retained FSR baseline, staged ADS/I2C recovery, and R2 MLX Fusion suspension are unchanged.
