#pragma once

#include <Arduino.h>
#include <esp_camera.h>

class CameraHardware
{
public:
    bool begin();
    bool selfTest();
    bool ready() const { return initialized; }
    bool psramReady() const { return psramAvailable; }
    camera_fb_t *capture();
    void release(camera_fb_t *frame);

private:
    bool initialized = false;
    bool psramAvailable = false;
};
