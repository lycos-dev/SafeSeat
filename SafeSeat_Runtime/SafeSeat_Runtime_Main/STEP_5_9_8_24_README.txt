SafeSeat Main Hub - Step 5.9.8.24

Purpose
-------
Prevent nuisance C1001 and camera ESP-NOW ON/OFF flapping when one or more
heartbeats are delayed or missed.

C1001 link health
-----------------
WAIT  : no valid C1001 packet has ever arrived.
ON    : packet age <= 4.5 s.
STALE : packet age > 4.5 s but <= 10 s. Remote evidence is withheld from Fusion.
OFF   : no valid packet for > 10 s.

Camera link health (prepared in advance)
----------------------------------------
WAIT  : no valid camera status/result packet has ever arrived.
ON    : packet age <= 5 s.
STALE : packet age > 5 s but <= 12 s. Camera evidence is not accepted by Fusion.
OFF   : no valid packet for > 12 s.

Why this is safer
-----------------
The UI no longer claims a disconnect after a single delayed packet. At the same
time, STALE evidence is not treated as current physiological/camera evidence.
A fresh valid packet immediately returns the transport to ON.
