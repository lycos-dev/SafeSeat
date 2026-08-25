SAFESEAT CAMERA V7 — COLLECTOR-PARITY PREVIEW FIX
====================================================

WHY V7 EXISTS
-------------
The V6 serial log proved that inference itself was fast (~225-248 ms) and
continued running, but the browser preview was reported as visually frozen and
the model was heavily biased toward leaning_right.

V7 does NOT retrain or change the verified INT8 model.

Instead it first removes the frame-pipeline uncertainty:

1. Camera settings now match the successful dataset collector exactly:
   - 320x240 QVGA JPEG
   - jpeg_quality 12
   - 2 PSRAM frame buffers
   - CAMERA_GRAB_LATEST
   - hmirror 0
   - vflip 0
   - 100 ms camera capture cadence

2. Browser preview is prioritized:
   - no-cache HTTP headers
   - no overlapping browser image requests
   - preview mutex released before slow Wi-Fi transmission
   - status polling reduced to once/second

3. ML is deliberately paced:
   - ~700 ms quiet gap after each inference
   - this leaves substantially more time for Wi-Fi/system work
   - the 5-frame verification vote still remains

4. Every inference proves which image it used:
   Serial now includes:
     srcFrame=<N>
     jpeg=<bytes>
     hash=0x........
     inputMAD=<value>

   srcFrame must keep increasing.
   hash should normally change.
   inputMAD should rise when the scene/posture changes.

5. Browser includes:
     Open latest exact ML source JPEG

   That image is the JPEG source actually used by the ML pipeline.

FIRST TEST
----------
Upload V7 and open Serial at 115200.

Connect:
  SSID: SafeSeat-Camera-Test
  PASS: SafeSeat123
  URL : http://192.168.4.1

Before testing poses:
- confirm the browser video visibly updates smoothly
- move one hand clearly and verify the preview follows it
- inspect Serial and confirm srcFrame increases

Then test:
  upright
  left
  right
  forward
  backward

Hold each pose for at least one completed 5-frame vote.

WHAT TO SEND BACK
-----------------
Send the Serial output covering:
- boot
- a few seconds of upright
- left
- right
- forward
- backward

If prediction still collapses to leaning_right while:
- srcFrame changes,
- hash changes,
- inputMAD changes,
then the issue is NOT a frozen frame pipeline. At that point we will capture
the exact /mlframe JPEGs and compare them against the desktop model/preprocessing.

DO NOT RETRAIN YET.
DO NOT ADD ESP-NOW YET.
