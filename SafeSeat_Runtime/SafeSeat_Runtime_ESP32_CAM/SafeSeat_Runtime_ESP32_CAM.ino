#include <Arduino.h>
#include <esp_camera.h>

#include "Config.h"
#include "CameraPins.h"
#include "CameraProtocol.h"
#include "CameraComm.h"
#include "PostureInference.h"

// ============================================================
// SAFESEAT ESP32-CAM VERIFICATION NODE - STEP 5.9.4
//
// Hardware:
//   AI-Thinker ESP32-CAM + OV2640
//   FTDI programming; 5V -> ESP32-CAM 5V input
//
// Normal operation:
//   ESP32 remains powered for ESP-NOW.
//   OV2640 is DEINITIALIZED / powered down between requests.
//
// Verification request:
//   Main Hub trigger -> camera wake -> 3-frame INT8 inference
//   -> majority posture -> ESP-NOW result -> camera off again.
// ============================================================

CameraComm cameraComm;
PostureInference postureInference;

bool commReady = false;
bool modelReady = false;
bool cameraSelfTestReady = false;
bool cameraBusy = false;

uint32_t resultSequence = 0;
uint32_t lastHandledRequestId = 0;
unsigned long lastInferenceMillis = 0;
unsigned long lastPrintMillis = 0;

bool initializeCamera()
{
    digitalWrite(PWDN_GPIO_NUM, LOW);
    delay(20);

    camera_config_t config{};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;

    // JPEG is intentionally used while Wi-Fi/ESP-NOW is active.
    // We decode each low-resolution frame to RGB888 before INT8
    // model preprocessing.
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QQVGA; // 160 x 120
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    const esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("[CAM] Camera init failed: 0x%X\n", static_cast<unsigned>(err));
        digitalWrite(PWDN_GPIO_NUM, HIGH);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != nullptr)
    {
        sensor->set_vflip(sensor, CAMERA_VERTICAL_FLIP ? 1 : 0);
        sensor->set_hmirror(sensor, CAMERA_HORIZONTAL_MIRROR ? 1 : 0);
    }

    return true;
}

void shutdownCamera()
{
    esp_camera_deinit();
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);
}

bool cameraSelfTest()
{
    if (!initializeCamera())
    {
        return false;
    }

    camera_fb_t *frame = esp_camera_fb_get();
    const bool ok = frame != nullptr && frame->buf != nullptr && frame->len > 0;

    if (frame != nullptr)
    {
        esp_camera_fb_return(frame);
    }

    shutdownCamera();
    return ok;
}

CameraResultPacket processVerification(const CameraTriggerPacket &trigger)
{
    CameraResultPacket packet;
    packet.requestId = trigger.requestId;
    packet.sequence = ++resultSequence;

    cameraBusy = true;
    const unsigned long fullStart = millis();

    bool captureError = false;
    bool inferenceError = false;

    if (!modelReady || !initializeCamera())
    {
        captureError = true;
        packet.flags |= CAMERA_RESULT_CAPTURE_ERROR;
        packet.inferenceMillis = millis() - fullStart;
        cameraBusy = false;
        return packet;
    }

    // Let exposure settle without using these frames as evidence.
    for (uint8_t i = 0; i < CAMERA_WARMUP_DISCARD_FRAMES; ++i)
    {
        camera_fb_t *warmup = esp_camera_fb_get();
        if (warmup != nullptr)
        {
            esp_camera_fb_return(warmup);
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
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == nullptr)
        {
            captureError = true;
        }
        else
        {
            const PostureInferenceResult inference = postureInference.infer(frame);
            esp_camera_fb_return(frame);

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

    shutdownCamera();

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
    delay(1000);

    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" SAFESEAT ESP32-CAM - STEP 5.9.4");
    Serial.println(" AI-Thinker | INT8 posture verification");
    Serial.println(" ESP-NOW trigger/result | Camera idle OFF");
    Serial.println("==========================================");

    Serial.print("PSRAM           : ");
    Serial.println(psramFound() ? "DETECTED" : "NOT DETECTED");

    modelReady = postureInference.begin();
    Serial.print("INT8 model      : ");
    Serial.println(modelReady ? "READY" : "FAILED");

    cameraSelfTestReady = cameraSelfTest();
    Serial.print("Camera self-test: ");
    Serial.println(cameraSelfTestReady ? "PASS" : "FAILED");

    commReady = cameraComm.begin();
    Serial.print("ESP-NOW         : ");
    Serial.println(commReady ? "READY" : "FAILED");

    Serial.println("Camera is now powered down until Main Hub requests verification.");
}

void loop()
{
    if (commReady)
    {
        cameraComm.update(
            modelReady,
            cameraSelfTestReady,
            postureInference.psramReady(),
            cameraBusy,
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
    if (now - lastPrintMillis >= 2000UL)
    {
        lastPrintMillis = now;
        const CameraNodeLinkStatus &link = cameraComm.getStatus();

        Serial.println();
        Serial.println("----------- CAMERA NODE STATUS ------------");
        Serial.print("Model ready     : "); Serial.println(modelReady ? "YES" : "NO");
        Serial.print("Camera self-test: "); Serial.println(cameraSelfTestReady ? "PASS" : "FAIL");
        Serial.print("Camera idle     : "); Serial.println(cameraBusy ? "NO - VERIFYING" : "YES - SENSOR OFF");
        Serial.print("Hub locked      : "); Serial.println(link.hubLocked ? "YES" : "SCANNING");
        Serial.print("Wi-Fi channel   : "); Serial.println(link.channel);
        Serial.print("Hub beacons     : "); Serial.println(link.hubBeaconsReceived);
        Serial.print("Triggers RX     : "); Serial.println(link.triggerPacketsReceived);
        Serial.print("Results sent    : "); Serial.println(link.resultsSent);
        Serial.print("Last request    : "); Serial.println(lastHandledRequestId);
    }

    delay(5);
}
