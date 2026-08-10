#pragma once

#include <Arduino.h>
#include <esp_camera.h>

#include "CameraProtocol.h"

struct PostureInferenceResult
{
    bool valid = false;
    CameraPostureClass posture = CameraPostureClass::UNKNOWN;
    float confidence = 0.0f;
    unsigned long inferenceMillis = 0;
};

class PostureInference
{
public:
    bool begin();
    bool ready() const { return initialized; }
    bool psramReady() const { return psramAvailable; }

    PostureInferenceResult infer(camera_fb_t *frame);

private:
    bool initialized = false;
    bool psramAvailable = false;

    uint8_t *tensorArena = nullptr;
    uint8_t *rgbBuffer = nullptr;

    void *interpreterOpaque = nullptr;
    void *inputTensorOpaque = nullptr;
    void *outputTensorOpaque = nullptr;

    int8_t quantizeLut[256]{};

    bool initializeInterpreter();
    bool prepareInput(camera_fb_t *frame);
};
