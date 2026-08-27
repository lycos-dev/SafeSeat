#include "coco_pose.hpp"
#include "dl_image_jpeg.hpp"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "camera_protocol.hpp"
#include "frame_quality.hpp"
#include "model_data.hpp"
#include "safeseat_pose_anomaly.hpp"
#include "safeseat_occupant_anchor.hpp"
#include "safeseat_temporal_filter.hpp"
#include "safeseat_verification.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <list>

static const char *TAG = "SafeSeatCamera";

// Confirmed SafeSeat ESP32-S3 WROOM OV2640 pin map.
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_D2 8
#define CAM_PIN_D1 9
#define CAM_PIN_D3 10
#define CAM_PIN_D0 11
#define CAM_PIN_D4 12
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_D7 16
#define CAM_PIN_D6 17
#define CAM_PIN_D5 18
#define CAM_PIN_RESET -1
#define CAM_PIN_PWDN -1

namespace {
constexpr char SAFESEAT_SSID[] = "SafeSeat";
constexpr char SAFESEAT_PASSWORD[] = "safeseat123";
constexpr uint8_t SAFESEAT_CHANNEL = 6;
constexpr uint8_t BROADCAST_MAC[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

constexpr int CALIBRATION_SAMPLES = 5;
constexpr int BURST_FRAMES = 3;
constexpr int VERIFY_MAX_ATTEMPTS = 3;
constexpr float MIN_BURST_SHARPNESS = 300.0f;
constexpr float CALIBRATION_MIN_PERSON_SCORE = 0.35f;
constexpr float RUNTIME_MIN_PERSON_SCORE = 0.35f;
constexpr float CALIBRATION_MAX_ABS_Z = 4.5f;
constexpr float CALIBRATION_MAX_RMS_Z = 2.0f;
constexpr uint32_t STATUS_INTERVAL_MS = 1000;

constexpr uint32_t BASELINE_MAGIC = 0x35565353U; // SSV5
constexpr uint16_t BASELINE_VERSION = 53;

struct __attribute__((packed)) BaselineBlob {
    uint32_t magic = BASELINE_MAGIC;
    uint16_t version = BASELINE_VERSION;
    uint16_t feature_count = safeseat_model::FEATURE_COUNT;
    uint32_t session_id = 0;
    float values[safeseat_model::FEATURE_COUNT] = {};
    float anchor[SAFESEAT_OCCUPANT_ANCHOR_COUNT] = {};
    uint16_t checksum = 0;
};

struct RuntimeState {
    bool model_ready = false;
    bool camera_ready = false;
    bool psram_ready = false;
    bool busy = false;
    bool baseline_ready = false;
    bool baseline_provisional = false;
    bool calibrating = false;
    bool session_active = false;
    uint8_t calibration_count = 0;
    uint32_t session_id = 0;
    uint32_t last_handled_request_id = 0;
    uint32_t last_inference_ms = 0;
};

struct PoseObservation {
    bool capture_ok = false;
    bool decoded = false;
    bool sharp_enough = false;
    bool exactly_one_person = false;
    bool selected_from_multiple = false;
    bool pose_valid = false;
    bool occupant_selected = false;
    bool selection_ambiguous = false;
    int background_rejected = 0;
    float anchor_match_cost = 0.0f;
    float anchor_area_scale = 0.0f;
    SafeSeatOccupantAnchor anchor_observation;
    float person_score = 0.0f;
    SafeSeatPoseResult fallback_result;
    float sharpness = 0.0f;
    int burst_decoded = 0;
    uint32_t inference_ms = 0;
    float features[safeseat_model::FEATURE_COUNT] = {};
};

SemaphoreHandle_t g_state_mutex = nullptr;
SemaphoreHandle_t g_cal_mutex = nullptr;
QueueHandle_t g_command_queue = nullptr;
RuntimeState g_state;
CameraResultPacket g_last_result_packet{};
bool g_has_last_result_packet = false;
SafeSeatTemporalFilter g_temporal;

float g_calibration_samples[CALIBRATION_SAMPLES][safeseat_model::FEATURE_COUNT] = {};
SafeSeatOccupantAnchor g_anchor_samples[CALIBRATION_SAMPLES] = {};
float g_baseline[safeseat_model::FEATURE_COUNT] = {};
SafeSeatOccupantAnchor g_anchor = {};
uint32_t g_status_sequence = 0;
uint32_t g_result_sequence = 0;

float wrap_angle_delta(float a)
{
    float x = std::fmod(a + 90.0f, 180.0f);
    if (x < 0.0f) x += 180.0f;
    return x - 90.0f;
}

uint16_t baseline_checksum(const BaselineBlob &blob)
{
    return camera_crc16(reinterpret_cast<const uint8_t *>(&blob), offsetof(BaselineBlob, checksum));
}

bool baseline_blob_valid(const BaselineBlob &blob)
{
    if (blob.magic != BASELINE_MAGIC
        || blob.version != BASELINE_VERSION
        || blob.feature_count != safeseat_model::FEATURE_COUNT
        || blob.session_id == 0
        || blob.checksum != baseline_checksum(blob)) return false;
    for (int i = 0; i < safeseat_model::FEATURE_COUNT; ++i)
        if (!std::isfinite(blob.values[i])) return false;
    for (int i = 0; i < SAFESEAT_OCCUPANT_ANCHOR_COUNT; ++i)
        if (!std::isfinite(blob.anchor[i])) return false;
    return blob.anchor[2] > 1e-6f;
}

void storage_init()
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);
}

bool save_baseline_nvs_locked()
{
    BaselineBlob blob;
    blob.session_id = g_state.session_id;
    std::memcpy(blob.values, g_baseline, sizeof(g_baseline));
    std::memcpy(blob.anchor, &g_anchor, sizeof(g_anchor));
    blob.checksum = 0;
    blob.checksum = baseline_checksum(blob);

    nvs_handle_t h;
    if (nvs_open("cam_pose", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_blob(h, "baseline_v53", &blob, sizeof(blob));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
}

void clear_baseline_nvs()
{
    nvs_handle_t h;
    if (nvs_open("cam_pose", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "baseline_v53");
        nvs_commit(h);
        nvs_close(h);
    }
}

bool load_baseline_nvs()
{
    nvs_handle_t h;
    if (nvs_open("cam_pose", NVS_READONLY, &h) != ESP_OK) return false;
    BaselineBlob blob{};
    size_t n = sizeof(blob);
    esp_err_t e = nvs_get_blob(h, "baseline_v53", &blob, &n);
    nvs_close(h);
    if (e != ESP_OK || n != sizeof(blob) || !baseline_blob_valid(blob)) return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.session_active = true;
        g_state.session_id = blob.session_id;
        g_state.baseline_ready = true;
        g_state.baseline_provisional = false;
        g_state.calibrating = false;
        g_state.calibration_count = CALIBRATION_SAMPLES;
        xSemaphoreGive(g_state_mutex);
    }
    std::memcpy(g_baseline, blob.values, sizeof(g_baseline));
    std::memcpy(&g_anchor, blob.anchor, sizeof(g_anchor));
    ESP_LOGI(TAG, "Loaded passenger baseline + seat-occupant anchor from NVS for session=%lu.", (unsigned long)blob.session_id);
    return true;
}

void set_busy(bool busy)
{
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.busy = busy;
        xSemaphoreGive(g_state_mutex);
    }
}

void set_last_inference(uint32_t ms)
{
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.last_inference_ms = ms;
        xSemaphoreGive(g_state_mutex);
    }
}

void begin_calibration(uint32_t session_id, uint32_t request_id)
{
    if (session_id == 0) return;

    if (xSemaphoreTake(g_cal_mutex, portMAX_DELAY) == pdTRUE) {
        std::memset(g_calibration_samples, 0, sizeof(g_calibration_samples));
        std::memset(g_anchor_samples, 0, sizeof(g_anchor_samples));
        std::memset(g_baseline, 0, sizeof(g_baseline));
        g_anchor = {};
        xSemaphoreGive(g_cal_mutex);
    }

    g_temporal.reset();
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_has_last_result_packet = false;
        g_state.session_active = true;
        g_state.session_id = session_id;
        g_state.baseline_ready = false;
        g_state.baseline_provisional = false;
        g_state.calibrating = true;
        g_state.calibration_count = 0;
        g_state.last_handled_request_id = request_id;
        xSemaphoreGive(g_state_mutex);
    }

    // Do not erase the previous NVS baseline here. It is tagged with its old
    // session ID and cannot be used by the new passenger. The old copy remains
    // recoverable if power is lost before the new 5-sample baseline completes.
    ESP_LOGI(TAG, "Passenger session %lu: upright calibration started.", (unsigned long)session_id);
}

void reset_session(uint32_t request_id)
{
    if (xSemaphoreTake(g_cal_mutex, portMAX_DELAY) == pdTRUE) {
        std::memset(g_calibration_samples, 0, sizeof(g_calibration_samples));
        std::memset(g_anchor_samples, 0, sizeof(g_anchor_samples));
        std::memset(g_baseline, 0, sizeof(g_baseline));
        g_anchor = {};
        xSemaphoreGive(g_cal_mutex);
    }
    clear_baseline_nvs();
    g_temporal.reset();

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_has_last_result_packet = false;
        const bool model_ready = g_state.model_ready;
        const bool camera_ready = g_state.camera_ready;
        const bool psram_ready = g_state.psram_ready;
        g_state = RuntimeState{};
        g_state.model_ready = model_ready;
        g_state.camera_ready = camera_ready;
        g_state.psram_ready = psram_ready;
        g_state.last_handled_request_id = request_id;
        xSemaphoreGive(g_state_mutex);
    }
    ESP_LOGI(TAG, "Passenger session cleared; camera returned to idle/no-baseline state.");
}

float median5(int feature)
{
    float values[CALIBRATION_SAMPLES];
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) values[i] = g_calibration_samples[i][feature];
    std::sort(values, values + CALIBRATION_SAMPLES);
    return values[CALIBRATION_SAMPLES / 2];
}

float anchor_median5(int feature)
{
    float values[CALIBRATION_SAMPLES];
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i)
        values[i] = reinterpret_cast<const float *>(&g_anchor_samples[i])[feature];
    std::sort(values, values + CALIBRATION_SAMPLES);
    return values[CALIBRATION_SAMPLES / 2];
}

enum class CalibrationUpdate { COLLECTED, ACCEPTED, DROPPED_OUTLIER };

CalibrationUpdate calibration_add(const float features[safeseat_model::FEATURE_COUNT], const SafeSeatOccupantAnchor &anchor_obs)
{
    if (xSemaphoreTake(g_cal_mutex, portMAX_DELAY) != pdTRUE) return CalibrationUpdate::COLLECTED;

    uint8_t count = 0;
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        count = g_state.calibration_count;
        xSemaphoreGive(g_state_mutex);
    }

    if (count < CALIBRATION_SAMPLES) {
        std::memcpy(g_calibration_samples[count], features, sizeof(g_calibration_samples[0]));
        g_anchor_samples[count] = anchor_obs;
        ++count;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            g_state.calibration_count = count;
            xSemaphoreGive(g_state_mutex);
        }
    }

    if (count < CALIBRATION_SAMPLES) {
        xSemaphoreGive(g_cal_mutex);
        return CalibrationUpdate::COLLECTED;
    }

    float med[safeseat_model::FEATURE_COUNT];
    for (int j = 0; j < safeseat_model::FEATURE_COUNT; ++j) med[j] = median5(j);

    int worst = -1;
    float worst_metric = -1.0f;
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        float sum2 = 0.0f, max_abs = 0.0f;
        for (int j = 0; j < safeseat_model::FEATURE_COUNT; ++j) {
            float d = g_calibration_samples[i][j] - med[j];
            if (j == 0) d = wrap_angle_delta(d);
            const float z = d / std::max(std::fabs(safeseat_model::SCALER_SCALE[j]), 1e-6f);
            sum2 += z * z;
            max_abs = std::max(max_abs, std::fabs(z));
        }
        const float rms = std::sqrt(sum2 / safeseat_model::FEATURE_COUNT);
        const float metric = std::max(max_abs / CALIBRATION_MAX_ABS_Z, rms / CALIBRATION_MAX_RMS_Z);
        if (metric > worst_metric) { worst_metric = metric; worst = i; }
    }

    if (worst_metric > 1.0f) {
        for (int i = worst; i < CALIBRATION_SAMPLES - 1; ++i) {
            std::memcpy(g_calibration_samples[i], g_calibration_samples[i + 1], sizeof(g_calibration_samples[0]));
            g_anchor_samples[i] = g_anchor_samples[i + 1];
        }
        count = CALIBRATION_SAMPLES - 1;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            g_state.calibration_count = count;
            xSemaphoreGive(g_state_mutex);
        }
        xSemaphoreGive(g_cal_mutex);
        ESP_LOGW(TAG, "Calibration outlier dropped; one more valid upright pose required.");
        return CalibrationUpdate::DROPPED_OUTLIER;
    }

    std::memcpy(g_baseline, med, sizeof(g_baseline));
    float *anchor_values = reinterpret_cast<float *>(&g_anchor);
    for (int j = 0; j < SAFESEAT_OCCUPANT_ANCHOR_COUNT; ++j) anchor_values[j] = anchor_median5(j);
    bool saved = false;
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.baseline_ready = true;
        g_state.baseline_provisional = false;
        g_state.calibrating = false;
        g_state.calibration_count = CALIBRATION_SAMPLES;
        saved = save_baseline_nvs_locked();
        xSemaphoreGive(g_state_mutex);
    }
    xSemaphoreGive(g_cal_mutex);
    g_temporal.reset();
    ESP_LOGI(TAG, "Upright calibration complete (5/5). Baseline + seat-occupant anchor saved=%s. Camera is now idle until verification.", saved ? "YES" : "NO");
    return CalibrationUpdate::ACCEPTED;
}

esp_err_t init_camera()
{
    camera_config_t c{};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = CAM_PIN_D0; c.pin_d1 = CAM_PIN_D1; c.pin_d2 = CAM_PIN_D2; c.pin_d3 = CAM_PIN_D3;
    c.pin_d4 = CAM_PIN_D4; c.pin_d5 = CAM_PIN_D5; c.pin_d6 = CAM_PIN_D6; c.pin_d7 = CAM_PIN_D7;
    c.pin_xclk = CAM_PIN_XCLK; c.pin_pclk = CAM_PIN_PCLK; c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF;
    c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC; c.pin_pwdn = CAM_PIN_PWDN; c.pin_reset = CAM_PIN_RESET;
    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 12;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    esp_err_t e = esp_camera_init(&c);
    if (e != ESP_OK) ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", e);
    return e;
}

bool ensure_broadcast_peer()
{
    if (esp_now_is_peer_exist(BROADCAST_MAC)) return true;
    esp_now_peer_info_t peer{};
    std::memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t e = esp_now_add_peer(&peer);
    return e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST;
}

void on_receive(const esp_now_recv_info_t *, const uint8_t *data, int len)
{
    if (!data || len != static_cast<int>(sizeof(CameraCommandPacket)) || !g_command_queue) return;
    CameraCommandPacket packet;
    std::memcpy(&packet, data, sizeof(packet));
    if (packet.magic != CAMERA_COMMAND_MAGIC
        || packet.version != CAMERA_WIRE_VERSION
        || packet.packetSize != sizeof(CameraCommandPacket)
        || packet.requestId == 0
        || packet.checksum != cameraCommandChecksum(packet)) return;

    // Queue retries too. VERIFY duplicates are cheap after completion because
    // handle_command() resends the cached result instead of rerunning YOLO.
    // A 16-entry queue safely absorbs Main Hub retries during a ~60 s two-pass
    // abnormal verification.
    xQueueSend(g_command_queue, &packet, 0);
}

void wifi_and_espnow_start()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t sta{};
    std::snprintf(reinterpret_cast<char *>(sta.sta.ssid), sizeof(sta.sta.ssid), "%s", SAFESEAT_SSID);
    std::snprintf(reinterpret_cast<char *>(sta.sta.password), sizeof(sta.sta.password), "%s", SAFESEAT_PASSWORD);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(SAFESEAT_CHANNEL, WIFI_SECOND_CHAN_NONE));
    esp_wifi_connect(); // ESP-NOW still works on channel 6 if association is temporarily unavailable.

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_receive));
    if (!ensure_broadcast_peer()) {
        ESP_LOGE(TAG, "Failed to add ESP-NOW broadcast peer.");
        abort();
    }
    ESP_LOGI(TAG, "ESP-NOW ready on SafeSeat channel %u; joining SSID '%s'.", SAFESEAT_CHANNEL, SAFESEAT_SSID);
}

void send_status()
{
    RuntimeState snapshot;
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE) return;
    snapshot = g_state;
    xSemaphoreGive(g_state_mutex);

    CameraStatusPacket packet;
    packet.sequence = ++g_status_sequence;
    packet.sessionId = snapshot.session_id;
    packet.lastHandledRequestId = snapshot.last_handled_request_id;
    packet.lastInferenceMillis = snapshot.last_inference_ms;
    packet.freeHeapBytes = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_8BIT));
    packet.calibrationCount = snapshot.calibration_count;
    packet.calibrationTarget = CALIBRATION_SAMPLES;
    if (snapshot.model_ready) packet.flags |= CAMERA_STATUS_MODEL_READY;
    if (snapshot.camera_ready) packet.flags |= CAMERA_STATUS_CAMERA_READY;
    if (snapshot.psram_ready) packet.flags |= CAMERA_STATUS_PSRAM_READY;
    if (snapshot.busy) packet.flags |= CAMERA_STATUS_BUSY;
    if (snapshot.baseline_ready) packet.flags |= CAMERA_STATUS_BASELINE_READY;
    if (snapshot.calibrating) packet.flags |= CAMERA_STATUS_CALIBRATING;
    if (snapshot.session_active) packet.flags |= CAMERA_STATUS_SESSION_ACTIVE;
    if (snapshot.baseline_provisional) packet.flags |= CAMERA_STATUS_BASELINE_PROVISIONAL;
    uint8_t primary = SAFESEAT_CHANNEL;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) packet.channel = primary;
    else packet.channel = SAFESEAT_CHANNEL;
    packet.checksum = 0;
    packet.checksum = cameraStatusChecksum(packet);
    esp_now_send(BROADCAST_MAC, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

void status_task(void *)
{
    while (true) {
        send_status();
        vTaskDelay(pdMS_TO_TICKS(STATUS_INTERVAL_MS));
    }
}

PoseObservation run_pose_once(COCOPose *pose, const float *baseline_for_fallback = nullptr,
                             const SafeSeatOccupantAnchor *tracked_anchor = nullptr,
                             bool calibration_mode = false)
{
    PoseObservation obs;
    set_busy(true);
    const int64_t start_us = esp_timer_get_time();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { set_busy(false); return obs; }
    obs.capture_ok = true;

    dl::image::jpeg_img_t jpg{.data = static_cast<void *>(fb->buf), .data_len = fb->len};
    auto best_img = dl::image::sw_decode_jpeg(jpg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (best_img.data) {
        obs.decoded = true; obs.burst_decoded = 1;
        obs.sharpness = safeseat_rgb888_sharpness(reinterpret_cast<const uint8_t *>(best_img.data), best_img.width, best_img.height);
    }
    esp_camera_fb_return(fb);

    for (int i = 1; i < BURST_FRAMES; ++i) {
        fb = esp_camera_fb_get(); if (!fb) continue;
        dl::image::jpeg_img_t candidate_jpg{.data = static_cast<void *>(fb->buf), .data_len = fb->len};
        auto candidate = dl::image::sw_decode_jpeg(candidate_jpg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
        if (candidate.data) {
            ++obs.burst_decoded;
            const float sharp = safeseat_rgb888_sharpness(reinterpret_cast<const uint8_t *>(candidate.data), candidate.width, candidate.height);
            if (!best_img.data || sharp > obs.sharpness) {
                if (best_img.data) heap_caps_free(best_img.data);
                best_img = candidate; candidate.data = nullptr; obs.sharpness = sharp; obs.decoded = true;
            }
            if (candidate.data) heap_caps_free(candidate.data);
        }
        esp_camera_fb_return(fb);
    }

    if (!best_img.data) { set_busy(false); return obs; }
    obs.sharp_enough = obs.sharpness >= MIN_BURST_SHARPNESS;
    if (obs.sharp_enough) {
        auto &poses = pose->run(best_img);
        SafeSeatOccupantSelection selection;
        if (calibration_mode)
            selection = safeseat_select_calibration_occupant(poses, best_img.width, best_img.height, CALIBRATION_MIN_PERSON_SCORE);
        else if (tracked_anchor)
            selection = safeseat_select_tracked_occupant(poses, best_img.width, best_img.height, *tracked_anchor, RUNTIME_MIN_PERSON_SCORE);
        else
            selection = safeseat_select_calibration_occupant(poses, best_img.width, best_img.height, RUNTIME_MIN_PERSON_SCORE);

        const dl::detect::result_t *selected = selection.pose;
        obs.selected_from_multiple = selection.selected_from_multiple;
        obs.selection_ambiguous = selection.ambiguous;
        obs.background_rejected = selection.rejected_background;
        obs.anchor_match_cost = selection.match_cost;
        obs.anchor_area_scale = selection.area_scale;
        if (selected) {
            obs.occupant_selected = true;
            obs.exactly_one_person = poses.size() == 1;
            obs.person_score = selected->score;
            safeseat_make_occupant_anchor_observation(*selected, best_img.width, best_img.height, obs.anchor_observation);
            obs.pose_valid = safeseat_extract_pose_features(*selected, best_img.width, best_img.height, obs.features);
            if (!obs.pose_valid && baseline_for_fallback)
                obs.fallback_result = safeseat_evaluate_missing_nose_forward_fallback(*selected, best_img.width, best_img.height, baseline_for_fallback);
            if (selection.selected_from_multiple)
                ESP_LOGI(TAG, "Seat-occupant association selected 1/%u detections; rejected_background=%d match_cost=%.3f area_x=%.3f.",
                         (unsigned)poses.size(), selection.rejected_background, selection.match_cost, selection.area_scale);
        } else if (!poses.empty()) {
            if (selection.ambiguous)
                ESP_LOGW(TAG, "Seat-occupant association ambiguous among %u detections; staying UNKNOWN.", (unsigned)poses.size());
            else
                ESP_LOGW(TAG, "No detection matched calibrated seat occupant; rejected %d background candidate(s); staying UNKNOWN.", selection.rejected_background);
        }
    }

    heap_caps_free(best_img.data);
    obs.inference_ms = static_cast<uint32_t>((esp_timer_get_time() - start_us) / 1000);
    set_last_inference(obs.inference_ms);
    set_busy(false);
    return obs;
}

uint16_t confidence_milli(float score)
{
    score = std::max(0.0f, std::min(1.0f, score));
    return static_cast<uint16_t>(std::lround(score * 1000.0f));
}

void send_result(uint32_t request_id, uint32_t session_id, CameraPostureClass decision,
                 const SafeSeatPoseResult &pose_result, uint8_t valid_observations,
                 float person_score, uint32_t inference_ms, uint8_t extra_flags = 0)
{
    CameraResultPacket packet;
    packet.requestId = request_id;
    packet.sessionId = session_id;
    packet.sequence = ++g_result_sequence;
    packet.postureClass = static_cast<uint8_t>(decision);
    packet.rawState = static_cast<uint8_t>(pose_result.state);
    packet.flags = extra_flags;
    if (decision == CameraPostureClass::UPRIGHT || decision == CameraPostureClass::NON_UPRIGHT)
        packet.flags |= CAMERA_RESULT_VALID;
    packet.validFrames = valid_observations;
    packet.confidenceMilli = confidence_milli(person_score);
    packet.ifScore = pose_result.if_score;
    packet.ocsvmScore = pose_result.ocsvm_score;
    packet.inferenceMillis = inference_ms;
    packet.checksum = 0;
    packet.checksum = cameraResultChecksum(packet);
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_last_result_packet = packet;
        g_has_last_result_packet = true;
        xSemaphoreGive(g_state_mutex);
    }
    esp_now_send(BROADCAST_MAC, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

void calibration_step(COCOPose *pose)
{
    uint32_t expected_session = 0;
    bool should_calibrate = false;
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        should_calibrate = g_state.session_active && g_state.calibrating && !g_state.baseline_ready;
        expected_session = g_state.session_id;
        xSemaphoreGive(g_state_mutex);
    }
    if (!should_calibrate || expected_session == 0) return;

    PoseObservation obs = run_pose_once(pose, nullptr, nullptr, true);

    // A RESET/CALIBRATE command can arrive during the ~28 s inference. Do not
    // let the old observation leak into a different passenger session.
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        should_calibrate = g_state.session_active && g_state.calibrating
            && !g_state.baseline_ready && g_state.session_id == expected_session;
        xSemaphoreGive(g_state_mutex);
    }
    if (!should_calibrate) return;

    if (!obs.sharp_enough) {
        ESP_LOGW(TAG, "CAL session=%lu: blur rejected (sharp=%.1f).", (unsigned long)expected_session, obs.sharpness);
        return;
    }
    if (!obs.pose_valid || obs.person_score < CALIBRATION_MIN_PERSON_SCORE) {
        ESP_LOGW(TAG, "CAL session=%lu: waiting for valid nose+shoulders pose (score=%.3f).", (unsigned long)expected_session, obs.person_score);
        return;
    }

    const CalibrationUpdate update = calibration_add(obs.features, obs.anchor_observation);
    uint8_t count = 0;
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        count = g_state.calibration_count;
        xSemaphoreGive(g_state_mutex);
    }
    if (update == CalibrationUpdate::DROPPED_OUTLIER)
        ESP_LOGW(TAG, "CAL session=%lu: unstable sample dropped; %u/5 accepted.", (unsigned long)expected_session, count);
    else
        ESP_LOGI(TAG, "CAL session=%lu: %u/5 valid upright poses.", (unsigned long)expected_session, count);
}

void verify_request(COCOPose *pose, const CameraCommandPacket &command)
{
    RuntimeState snapshot;
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE) return;
    snapshot = g_state;
    xSemaphoreGive(g_state_mutex);

    SafeSeatPoseResult blank_result;
    if (!snapshot.session_active
        || snapshot.session_id != command.sessionId
        || !snapshot.baseline_ready
        || !snapshot.model_ready
        || !snapshot.camera_ready
        || !snapshot.psram_ready)
    {
        send_result(command.requestId, command.sessionId, CameraPostureClass::NOT_READY,
                    blank_result, 0, 0.0f, 0);
        return;
    }

    float baseline[safeseat_model::FEATURE_COUNT];
    SafeSeatOccupantAnchor anchor;
    if (xSemaphoreTake(g_cal_mutex, portMAX_DELAY) == pdTRUE) {
        std::memcpy(baseline, g_baseline, sizeof(baseline));
        anchor = g_anchor;
        xSemaphoreGive(g_cal_mutex);
    }

    g_temporal.reset();
    uint32_t cumulative_ms = 0;
    uint8_t valid_observations = 0;
    SafeSeatPoseResult last_result;
    float last_person_score = 0.0f;

    ESP_LOGI(TAG, "VERIFY request=%lu session=%lu started.",
             (unsigned long)command.requestId, (unsigned long)command.sessionId);

    for (int attempt = 0; attempt < VERIFY_MAX_ATTEMPTS; ++attempt) {
        PoseObservation obs = run_pose_once(pose, baseline, &anchor, false);
        cumulative_ms += obs.inference_ms;

        RuntimeState current;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            current = g_state;
            xSemaphoreGive(g_state_mutex);
        }
        if (!current.session_active || current.session_id != command.sessionId) {
            ESP_LOGW(TAG, "VERIFY request=%lu aborted: passenger session changed.", (unsigned long)command.requestId);
            return;
        }

        if (!obs.sharp_enough || obs.person_score < RUNTIME_MIN_PERSON_SCORE) {
            const SafeSeatTemporalStatus temporal = g_temporal.update(false, SafeSeatPoseState::UNKNOWN);
            const SafeSeatVerificationState contract = safeseat_verification_decide(
                true, true, false, SafeSeatPoseState::UNKNOWN, temporal.state);
            ESP_LOGW(TAG, "VERIFY request=%lu attempt=%d UNKNOWN (sharp=%.1f score=%.3f) contract=%s | UNKNOWN NEVER CLEARS.",
                     (unsigned long)command.requestId, attempt + 1, obs.sharpness, obs.person_score,
                     safeseat_verification_state_name(contract));
            continue;
        }

        last_person_score = obs.person_score;
        if (obs.pose_valid) last_result = safeseat_evaluate_pose(obs.features, baseline);
        else last_result = obs.fallback_result;
        if (!last_result.valid_pose) {
            const SafeSeatTemporalStatus temporal = g_temporal.update(false, SafeSeatPoseState::UNKNOWN);
            const SafeSeatVerificationState contract = safeseat_verification_decide(
                true, true, false, SafeSeatPoseState::UNKNOWN, temporal.state);
            ESP_LOGW(TAG, "VERIFY request=%lu attempt=%d UNKNOWN: no full pose and forward fallback did not pass; contract=%s | UNKNOWN NEVER CLEARS.",
                     (unsigned long)command.requestId, attempt + 1,
                     safeseat_verification_state_name(contract));
            continue;
        }

        ++valid_observations;
        SafeSeatTemporalStatus temporal = g_temporal.update(true, last_result.state);
        const SafeSeatVerificationState contract = safeseat_verification_decide(
            true, true, last_result.valid_pose, last_result.state, temporal.state);
        if (last_result.fallback_used) {
            ESP_LOGW(TAG, "VERIFY request=%lu attempt=%d FORWARD_FALLBACK shoulder_x=%.3f box_x=%.3f.",
                     (unsigned long)command.requestId, attempt + 1,
                     last_result.fallback_shoulder_ratio, last_result.fallback_box_ratio);
        }

        if (contract == SafeSeatVerificationState::HOLD_DEVIATION
            && temporal.state == SafeSeatFilteredState::DEVIATION_CONFIRMED) {
            send_result(command.requestId, command.sessionId, CameraPostureClass::NON_UPRIGHT,
                        last_result, valid_observations, last_person_score, cumulative_ms);
            ESP_LOGI(TAG, "VERIFY request=%lu -> NON_UPRIGHT_CONFIRMED in %lums.",
                     (unsigned long)command.requestId, (unsigned long)cumulative_ms);
            return;
        }

        // V5.3.2 mirrors the standalone V4.3.2 verification-safe contract:
        // UNKNOWN can never clear a sensor-triggered emergency. UPRIGHT is
        // permitted only when the CURRENT raw observation is explicitly NORMAL,
        // the temporal filter is NORMAL, and the integrated runtime has two
        // consecutive clean NORMAL observations with no abnormal streak.
        if (contract == SafeSeatVerificationState::CLEAR_UPRIGHT
            && temporal.normal_streak >= 2
            && temporal.abnormal_streak == 0)
        {
            send_result(command.requestId, command.sessionId, CameraPostureClass::UPRIGHT,
                        last_result, valid_observations, last_person_score, cumulative_ms);
            ESP_LOGI(TAG, "VERIFY request=%lu -> UPRIGHT_CONFIRMED after %d clean normals in %lums.",
                     (unsigned long)command.requestId, temporal.normal_streak, (unsigned long)cumulative_ms);
            return;
        }

        send_result(command.requestId, command.sessionId, CameraPostureClass::DEVIATION_PENDING,
                    last_result, valid_observations, last_person_score, cumulative_ms);
        ESP_LOGI(TAG, "VERIFY request=%lu -> PENDING/MIXED contract=%s; collecting confirmation.",
                 (unsigned long)command.requestId, safeseat_verification_state_name(contract));
    }

    send_result(command.requestId, command.sessionId, CameraPostureClass::UNKNOWN,
                last_result, valid_observations, last_person_score, cumulative_ms);
    ESP_LOGW(TAG, "VERIFY request=%lu -> UNKNOWN after %d attempts | UNKNOWN NEVER CLEARS.",
             (unsigned long)command.requestId, VERIFY_MAX_ATTEMPTS);
}

void handle_command(COCOPose *pose, const CameraCommandPacket &command)
{
    const CameraCommandType type = static_cast<CameraCommandType>(command.command);

    if (type == CameraCommandType::CALIBRATE_UPRIGHT) {
        RuntimeState current;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            current = g_state;
            xSemaphoreGive(g_state_mutex);
        }
        if (command.sessionId != 0
            && current.session_active
            && current.session_id == command.sessionId
            && (current.calibrating || current.baseline_ready))
        {
            if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
                g_state.last_handled_request_id = command.requestId;
                xSemaphoreGive(g_state_mutex);
            }
        }
        else {
            begin_calibration(command.sessionId, command.requestId);
        }
        return;
    }

    if (type == CameraCommandType::RESET_SESSION) {
        RuntimeState current;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            current = g_state;
            xSemaphoreGive(g_state_mutex);
        }
        if (command.sessionId == 0 || !current.session_active || current.session_id == command.sessionId)
            reset_session(command.requestId);
        return;
    }

    if (type == CameraCommandType::VERIFY_POSTURE) {
        CameraResultPacket cached{};
        bool resend_cached = false;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            g_state.last_handled_request_id = command.requestId;
            resend_cached = g_has_last_result_packet
                && g_last_result_packet.requestId == command.requestId
                && g_last_result_packet.sessionId == command.sessionId;
            if (resend_cached) cached = g_last_result_packet;
            xSemaphoreGive(g_state_mutex);
        }
        if (resend_cached) {
            esp_now_send(BROADCAST_MAC, reinterpret_cast<const uint8_t *>(&cached), sizeof(cached));
            ESP_LOGI(TAG, "VERIFY request=%lu duplicate -> cached result resent.", (unsigned long)command.requestId);
            return;
        }
        verify_request(pose, command);
        return;
    }

    // PING/CANCEL are lightweight acknowledgements. A late result from a
    // canceled request is still rejected by Main Hub transaction IDs.
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.last_handled_request_id = command.requestId;
        xSemaphoreGive(g_state_mutex);
    }
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "====================================================");
    ESP_LOGI(TAG, "SafeSeat ESP32-S3 Camera V5.3.2 - ESP-NOW VERIFICATION-SAFE Integrated");
    ESP_LOGI(TAG, "V4.3.2 camera contract: occupant lock + forward fallback + temporal filter + UNKNOWN never clears");
    ESP_LOGI(TAG, "====================================================");

    g_state_mutex = xSemaphoreCreateMutex();
    g_cal_mutex = xSemaphoreCreateMutex();
    g_command_queue = xQueueCreate(16, sizeof(CameraCommandPacket));
    if (!g_state_mutex || !g_cal_mutex || !g_command_queue) {
        ESP_LOGE(TAG, "Failed to create runtime synchronization objects.");
        return;
    }

    storage_init();
    wifi_and_espnow_start();

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.psram_ready = heap_caps_get_total_size(MALLOC_CAP_SPIRAM) >= (4u * 1024u * 1024u);
        xSemaphoreGive(g_state_mutex);
    }

    const esp_err_t camera_err = init_camera();
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.camera_ready = camera_err == ESP_OK;
        xSemaphoreGive(g_state_mutex);
    }
    if (camera_err != ESP_OK) return;

    COCOPose *pose = new COCOPose(COCOPose::YOLO11N_POSE_S8_V2);
    if (!pose) {
        ESP_LOGE(TAG, "Failed to allocate COCOPose.");
        return;
    }
    pose->set_score_thr(0.25f);
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.model_ready = true;
        xSemaphoreGive(g_state_mutex);
    }

    load_baseline_nvs();

    xTaskCreatePinnedToCore(status_task, "camera_status", 4096, nullptr, 5, nullptr, 1);
    ESP_LOGI(TAG, "Camera ready. Idle until Main Hub passenger-session command.");

    while (true) {
        CameraCommandPacket command;
        if (xQueueReceive(g_command_queue, &command, pdMS_TO_TICKS(100)) == pdTRUE) {
            handle_command(pose, command);
            continue;
        }

        bool calibrating = false;
        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
            calibrating = g_state.session_active && g_state.calibrating && !g_state.baseline_ready;
            xSemaphoreGive(g_state_mutex);
        }
        if (calibrating) {
            calibration_step(pose); // one ~28 s sample, then return to command queue
            continue;
        }

        // True idle: no continuous pose inference. Sensor Fusion on the Main
        // Hub is the trigger authority for later VERIFY_POSTURE commands.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
