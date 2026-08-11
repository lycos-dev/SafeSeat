#include "CameraHardware.h"

#include "CameraPins.h"
#include "Config.h"

bool CameraHardware::begin()
{
    if (initialized)
    {
        return true;
    }

    psramAvailable = psramFound();

    Serial.println();
    Serial.println("========================================");
    Serial.println(" INITIALIZING ESP32-S3 CAMERA");
    Serial.println("========================================");
    Serial.print("[PSRAM] ");
    Serial.println(psramAvailable ? "Detected" : "NOT detected");
    if (psramAvailable)
    {
        Serial.print("[PSRAM] Size: ");
        Serial.print(ESP.getPsramSize() / (1024 * 1024));
        Serial.println(" MB");
    }

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

    // Retained from the user's confirmed-working diagnostic sketch.
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Production inference does not need a VGA stream.  QQVGA is
    // captured as JPEG and resized to the model's 160x160 input.
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;

    if (psramAvailable)
    {
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    Serial.println("[CAMERA] Calling esp_camera_init()...");
    const esp_err_t error = esp_camera_init(&config);
    if (error != ESP_OK)
    {
        Serial.print("[ERROR] Camera initialization failed: 0x");
        Serial.println(static_cast<unsigned>(error), HEX);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == nullptr)
    {
        Serial.println("[ERROR] Camera sensor pointer is NULL.");
        esp_camera_deinit();
        return false;
    }

    // Explicitly freeze the runtime size/orientation after init.
    sensor->set_framesize(sensor, FRAMESIZE_QQVGA);
    sensor->set_vflip(sensor, CAMERA_VERTICAL_FLIP ? 1 : 0);
    sensor->set_hmirror(sensor, CAMERA_HORIZONTAL_MIRROR ? 1 : 0);

    Serial.print("[CAMERA] Sensor PID: 0x");
    Serial.println(sensor->id.PID, HEX);

    initialized = true;
    Serial.println("[CAMERA] Initialization successful.");
    return true;
}

camera_fb_t *CameraHardware::capture()
{
    if (!initialized)
    {
        return nullptr;
    }

    return esp_camera_fb_get();
}

void CameraHardware::release(camera_fb_t *frame)
{
    if (frame != nullptr)
    {
        esp_camera_fb_return(frame);
    }
}

bool CameraHardware::selfTest()
{
    if (!initialized && !begin())
    {
        return false;
    }

    delay(350);
    camera_fb_t *frame = capture();
    const bool ok = frame != nullptr
        && frame->buf != nullptr
        && frame->len > 0
        && frame->width > 0
        && frame->height > 0;

    if (ok)
    {
        Serial.print("[CAMERA] Self-test frame: ");
        Serial.print(frame->width);
        Serial.print(" x ");
        Serial.print(frame->height);
        Serial.print(" | JPEG ");
        Serial.print(frame->len);
        Serial.println(" bytes");
    }
    else
    {
        Serial.println("[ERROR] Camera self-test capture failed.");
    }

    release(frame);
    return ok;
}
