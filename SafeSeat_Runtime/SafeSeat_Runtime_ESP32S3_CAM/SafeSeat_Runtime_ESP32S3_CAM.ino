#include <Arduino.h>

#include "Config.h"
#include "CameraProtocol.h"
#include "CameraHardware.h"
#include "CameraComm.h"
#include "SafeSeatWiFi.h"
#include "PostureInference.h"

// ============================================================
// SAFESEAT ESP32-S3 WROOM CAMERA VERIFICATION NODE - STEP 5.9.5
//
// Hardware baseline:
//   ESP32-S3 WROOM camera board
//   exact pin map from the user's confirmed-working diagnostic
//   USB-UART upload + Serial Monitor
//
// Network roles:
//   ESP-NOW = Main trigger/status/result transport
//   Wi-Fi STA = joins Main Hub's future "SafeSeat" SoftAP
//
// Camera policy:
//   PWDN is not wired on this board (PWDN=-1), so the camera driver
//   remains initialized for reliability.  No frames are captured and
//   no CNN inference runs until Main sends a verification request.
// ============================================================

CameraHardware cameraHardware;
CameraComm cameraComm;
SafeSeatWiFi safeSeatWiFi;
PostureInference postureInference;

bool commReady = false;
bool modelReady = false;
bool cameraReady = false;
bool cameraSelfTestReady = false;
bool cameraBusy = false;

uint32_t resultSequence = 0;
uint32_t lastHandledRequestId = 0;
unsigned long lastInferenceMillis = 0;
unsigned long lastPrintMillis = 0;

CameraResultPacket processVerification(const CameraTriggerPacket &trigger)
{
    CameraResultPacket packet;
    packet.requestId = trigger.requestId;
    packet.sequence = ++resultSequence;

    cameraBusy = true;
    const unsigned long fullStart = millis();

    bool captureError = false;
    bool inferenceError = false;

    if (!modelReady || !cameraReady)
    {
        if (!cameraReady)
        {
            packet.flags |= CAMERA_RESULT_CAPTURE_ERROR;
        }
        if (!modelReady)
        {
            packet.flags |= CAMERA_RESULT_INFERENCE_ERROR;
        }
        packet.inferenceMillis = millis() - fullStart;
        cameraBusy = false;
        return packet;
    }

    // Exposure warm-up only when verification is requested.
    for (uint8_t i = 0; i < CAMERA_WARMUP_DISCARD_FRAMES; ++i)
    {
        camera_fb_t *warmup = cameraHardware.capture();
        if (warmup != nullptr)
        {
            cameraHardware.release(warmup);
        }
        else
        {
            captureError = true;
        }
        delay(80);
    }

    const uint8_t requestedFrames =
        trigger.frameCount > 0 && trigger.frameCount <= 5
            ? trigger.frameCount
            : CAMERA_VERIFY_FRAME_COUNT;

    const uint8_t requiredFrames =
        trigger.minValidFrames > 0 && trigger.minValidFrames <= requestedFrames
            ? trigger.minValidFrames
            : CAMERA_VERIFY_MIN_VALID_FRAMES;

    uint8_t votes[CAMERA_MODEL_CLASS_COUNT]{};
    float confidenceSum[CAMERA_MODEL_CLASS_COUNT]{};
    uint8_t validFrames = 0;
    unsigned long pureInferenceTotal = 0;

    for (uint8_t frameIndex = 0; frameIndex < requestedFrames; ++frameIndex)
    {
        camera_fb_t *frame = cameraHardware.capture();
        if (frame == nullptr)
        {
            captureError = true;
        }
        else
        {
            const PostureInferenceResult inference = postureInference.infer(frame);
            cameraHardware.release(frame);

            if (inference.valid)
            {
                const uint8_t cls = static_cast<uint8_t>(inference.posture);
                if (cls < CAMERA_MODEL_CLASS_COUNT)
                {
                    votes[cls]++;
                    confidenceSum[cls] += inference.confidence;
                    validFrames++;
                    pureInferenceTotal += inference.inferenceMillis;
                }
            }
            else
            {
                inferenceError = true;
            }
        }

        if (frameIndex + 1 < requestedFrames)
        {
            delay(CAMERA_INTER_FRAME_DELAY_MS);
        }
    }

    packet.validFrames = validFrames;

    if (captureError)
    {
        packet.flags |= CAMERA_RESULT_CAPTURE_ERROR;
    }
    if (inferenceError)
    {
        packet.flags |= CAMERA_RESULT_INFERENCE_ERROR;
    }

    if (validFrames >= requiredFrames)
    {
        int bestClass = 0;
        for (int cls = 1; cls < CAMERA_MODEL_CLASS_COUNT; ++cls)
        {
            if (votes[cls] > votes[bestClass]
                || (votes[cls] == votes[bestClass]
                    && confidenceSum[cls] > confidenceSum[bestClass]))
            {
                bestClass = cls;
            }
        }

        const float voteFraction =
            static_cast<float>(votes[bestClass]) / validFrames;
        const float meanWinnerConfidence =
            votes[bestClass] > 0
                ? confidenceSum[bestClass] / votes[bestClass]
                : 0.0f;
        const float consensusConfidence =
            voteFraction * meanWinnerConfidence;

        packet.postureClass = static_cast<uint8_t>(bestClass);
        packet.confidenceMilli = static_cast<uint16_t>(
            constrain(
                lroundf(consensusConfidence * 1000.0f),
                0L,
                1000L
            )
        );
        packet.flags |= CAMERA_RESULT_VALID;
    }

    packet.inferenceMillis = millis() - fullStart;
    lastInferenceMillis = packet.inferenceMillis;
    cameraBusy = false;

    Serial.println();
    Serial.println("[CAM] Verification complete");
    Serial.print("  Request ID     : "); Serial.println(packet.requestId);
    Serial.print("  Valid frames   : "); Serial.print(validFrames); Serial.print("/"); Serial.println(requestedFrames);
    Serial.print("  Pure infer ms  : "); Serial.println(pureInferenceTotal);
    Serial.print("  Full verify ms : "); Serial.println(packet.inferenceMillis);

    if (packet.flags & CAMERA_RESULT_VALID)
    {
        const auto posture = static_cast<CameraPostureClass>(packet.postureClass);
        Serial.print("  Posture        : "); Serial.println(cameraPostureText(posture));
        Serial.print("  Confidence     : "); Serial.print(packet.confidenceMilli / 10.0f, 1); Serial.println("%");
        Serial.print("  Fusion meaning : ");
        Serial.println(cameraPostureIsNormal(posture) ? "NORMAL / UPRIGHT" : "ABNORMAL / LEANING");
    }
    else
    {
        Serial.println("  Result         : INVALID - Main Hub must not use as camera evidence");
    }

    return packet;
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" SAFESEAT CAMERA - STEP 5.9.5");
    Serial.println(" ESP32-S3 WROOM | INT8 posture verifier");
    Serial.println(" ESP-NOW + future SafeSeat Wi-Fi STA");
    Serial.println("==========================================");

    cameraReady = cameraHardware.begin();
    cameraSelfTestReady = cameraReady && cameraHardware.selfTest();
    Serial.print("Camera hardware : ");
    Serial.println(cameraSelfTestReady ? "READY / SELF-TEST PASS" : "FAILED");

    modelReady = postureInference.begin();
    Serial.print("INT8 model      : ");
    Serial.println(modelReady ? "READY" : "FAILED");

    commReady = cameraComm.begin();
    Serial.print("ESP-NOW         : ");
    Serial.println(commReady ? "READY" : "FAILED");

    safeSeatWiFi.begin();

    Serial.println();
    Serial.println("[CAM] Event policy: no capture/inference until Main requests verification.");
    Serial.println("[CAM] PWDN is not wired on this S3 board, so camera remains initialized.");
}

void loop()
{
    safeSeatWiFi.update();

    if (commReady)
    {
        cameraComm.update(
            modelReady,
            cameraSelfTestReady,
            postureInference.psramReady(),
            cameraBusy,
            safeSeatWiFi.connected(),
            safeSeatWiFi.radioOwnsChannel(),
            lastInferenceMillis,
            lastHandledRequestId
        );
    }

    CameraTriggerPacket trigger;
    if (commReady && cameraComm.takeTrigger(trigger))
    {
        Serial.println();
        Serial.print("[CAM] Verification request received: ");
        Serial.println(trigger.requestId);

        CameraResultPacket result = processVerification(trigger);
        lastHandledRequestId = trigger.requestId;

        const bool sent = cameraComm.sendResult(result);
        Serial.print("[CAM] Result ESP-NOW send: ");
        Serial.println(sent ? "QUEUED" : "FAILED");
    }

    const unsigned long now = millis();
    if (now - lastPrintMillis >= 3000UL)
    {
        lastPrintMillis = now;
        const CameraNodeLinkStatus &link = cameraComm.getStatus();
        const SafeSeatWiFiStatus &wifi = safeSeatWiFi.getStatus();

        Serial.println();
        Serial.println("--------- ESP32-S3 CAMERA NODE STATUS ---------");
        Serial.print("Camera ready    : "); Serial.println(cameraSelfTestReady ? "YES" : "NO");
        Serial.print("Model ready     : "); Serial.println(modelReady ? "YES" : "NO");
        Serial.print("Verification    : "); Serial.println(cameraBusy ? "RUNNING" : "IDLE");
        Serial.print("SafeSeat Wi-Fi  : ");
        if (wifi.connected) Serial.println("CONNECTED");
        else if (wifi.connecting) Serial.println("CONNECTING");
        else Serial.println("WAITING / RETRY LATER");
        if (wifi.connected)
        {
            Serial.print("Camera IP       : "); Serial.println(WiFi.localIP());
        }
        Serial.print("Hub locked      : "); Serial.println(link.hubLocked ? "YES" : "SCANNING / AP-MANAGED");
        Serial.print("Radio channel   : "); Serial.println(link.channel);
        Serial.print("Hub beacons     : "); Serial.println(link.hubBeaconsReceived);
        Serial.print("Triggers RX     : "); Serial.println(link.triggerPacketsReceived);
        Serial.print("Results sent    : "); Serial.println(link.resultsSent);
        Serial.print("Last request    : "); Serial.println(lastHandledRequestId);
    }

    delay(5);
}
