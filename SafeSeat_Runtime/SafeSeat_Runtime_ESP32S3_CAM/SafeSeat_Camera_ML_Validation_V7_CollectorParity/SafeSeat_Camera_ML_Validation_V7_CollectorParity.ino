/*
 * SafeSeat ESP32-S3 Camera ML Validation V7 — Collector Parity
 *
 * PURPOSE:
 *   Fix/diagnose browser preview freezing while preserving the verified
 *   SafeSeat 64x64 FULL-INT8 posture model.
 *
 * CAMERA PATH NOW MATCHES THE DATASET COLLECTOR:
 *   QVGA JPEG 320x240
 *   jpeg_quality = 12
 *   fb_count = 2
 *   CAMERA_GRAB_LATEST
 *   100 ms capture cadence
 *   hmirror=0, vflip=0
 *
 * IMPORTANT:
 *   - Browser preview is PRIORITIZED.
 *   - ML runs more gently so Wi-Fi is not continuously pressured.
 *   - Every ML inference logs the exact source frame ID + JPEG fingerprint.
 *   - inputMAD reports whether the actual 64x64 tensor changed.
 *   - /mlframe shows the exact JPEG source used by the latest ML inference.
 *
 * MODEL:
 *   Input : [1,64,64,3] INT8
 *   Output: [1,5] INT8
 *   0 backward, 1 forward, 2 left, 3 right, 4 upright
 *   RGB / 255.0
 *   5-frame averaged vote
 *
 * Wi-Fi:
 *   SSID: SafeSeat-Camera-Test
 *   PASS: SafeSeat123
 *   URL : http://192.168.4.1
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "esp_task_wdt.h"

#include "CameraPins.h"
#include "CameraModelData.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ============================================================
// Wi-Fi
// ============================================================

static const char* AP_SSID = "SafeSeat-Camera-Test";
static const char* AP_PASSWORD = "SafeSeat123";

WebServer server(80);

// ============================================================
// Model contract
// ============================================================

static constexpr int MODEL_W = 64;
static constexpr int MODEL_H = 64;
static constexpr int MODEL_C = 3;
static constexpr int CLASS_COUNT = 5;

static constexpr unsigned int EXPECTED_MODEL_BYTES = 117824U;

static constexpr size_t TENSOR_ARENA_BYTES =
    1U * 1024U * 1024U;

static constexpr size_t RGB888_BYTES =
    320U * 240U * 3U;

static constexpr size_t JPEG_BUFFER_CAPACITY =
    200U * 1024U;

// Preview capture cadence (~4 FPS target).
static constexpr unsigned long CAMERA_CAPTURE_INTERVAL_MS = 100UL;

// After one inference finishes, request the most recent camera snapshot.
// Because inference itself is ~20 s, there is no need to request faster.
static constexpr unsigned long ML_MIN_GAP_MS = 700UL;

// Set 1 only if training used square center crop before resize.
#define SAFESEAT_CENTER_CROP_BEFORE_RESIZE 0

// ============================================================
// Classes
// ============================================================

enum PostureClass : int
{
    LEANING_BACKWARD = 0,
    LEANING_FORWARD = 1,
    LEANING_LEFT = 2,
    LEANING_RIGHT = 3,
    UPRIGHT = 4
};

static const char* CLASS_NAMES[CLASS_COUNT] =
{
    "leaning_backward",
    "leaning_forward",
    "leaning_left",
    "leaning_right",
    "upright"
};

static const char* SHORT_NAMES[CLASS_COUNT] =
{
    "BACK", "FWD", "LEFT", "RIGHT", "UP"
};

// ============================================================
// TFLM
// ============================================================

static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* inputTensor = nullptr;
static TfLiteTensor* outputTensor = nullptr;

static tflite::MicroMutableOpResolver<4> resolver;

static uint8_t* tensorArena = nullptr;
static uint8_t* rgb888Buffer = nullptr;

// ============================================================
// Shared JPEG buffers
// ============================================================

// Browser preview buffer.
static uint8_t* previewJpeg = nullptr;
static size_t previewJpegLen = 0;

// Dedicated HTTP send copy. The preview mutex is released BEFORE the
// potentially slow Wi-Fi client write so the camera task can keep updating.
static uint8_t* httpJpeg = nullptr;
static size_t httpJpegLen = 0;

// Dedicated ML snapshot buffer.
// Camera task writes only when no ML snapshot is pending.
static uint8_t* mlJpeg = nullptr;
static size_t mlJpegLen = 0;

static SemaphoreHandle_t previewMutex = nullptr;
static SemaphoreHandle_t mlJpegMutex = nullptr;

static volatile bool mlSnapshotPending = false;

// Exact frame identity used by ML.
static volatile uint32_t mlSnapshotFrameId = 0;
static volatile uint32_t latestInferenceSourceFrameId = 0;
static volatile uint32_t latestInferenceJpegHash = 0;
static volatile size_t latestInferenceJpegLen = 0;

// Direct tensor-change diagnostic.
static int8_t previousModelInput[64 * 64 * 3] = {0};
static bool previousModelInputValid = false;
static float latestInputMAD = 0.0f;

// Prevent the camera task from queueing the NEXT ML image while the
// current ~20-23 s inference is still running. This ensures each new
// inference starts from a fresh camera frame rather than a snapshot
// captured one inference cycle earlier.
static volatile bool mlInferenceBusy = false;

// ============================================================
// Task handles
// ============================================================

static TaskHandle_t cameraTaskHandle = nullptr;
static TaskHandle_t mlTaskHandle = nullptr;

// ============================================================
// Runtime state
// ============================================================

static volatile bool cameraReady = false;
static volatile bool modelReady = false;

static volatile uint32_t frameCounter = 0;
static volatile uint32_t inferenceCounter = 0;
static volatile uint32_t captureFailures = 0;
static volatile uint32_t decodeFailures = 0;

static volatile unsigned long latestFrameAt = 0;
static volatile unsigned long latestInferenceAt = 0;
static volatile unsigned long latestInferenceMs = 0;

static volatile int latestPrediction = -1;
static float latestConfidence = 0.0f;
static float latestProbabilities[CLASS_COUNT] = {0};

static volatile int expectedClass = -1;
static volatile bool recordSamples = false;

static uint32_t confusion[CLASS_COUNT][CLASS_COUNT] = {{0}};

// Deployment verification vote: 5 consecutive frame probabilities.
// A vote is emitted only after 5 fresh ML inferences.
static constexpr int VOTE_FRAMES = 5;
static float voteAccumulator[CLASS_COUNT] = {0};
static int voteFrameCount = 0;
static volatile uint32_t voteCounter = 0;
static volatile int latestVotedPrediction = -1;
static float latestVotedConfidence = 0.0f;
static float latestVotedProbabilities[CLASS_COUNT] = {0};

// Protect result/probability/confusion access.
static SemaphoreHandle_t resultMutex = nullptr;

// ============================================================
// Browser UI
// ============================================================

static const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SafeSeat Camera ML</title>
<style>
body{margin:0;background:#111;color:#eee;font-family:Arial,sans-serif}
.wrap{max-width:900px;margin:auto;padding:14px}
.card{background:#1b1b1b;border-radius:14px;padding:14px;margin-bottom:14px}
.cam{position:relative;width:100%;aspect-ratio:4/3;background:#000;border-radius:12px;overflow:hidden}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
.guide{position:absolute;left:17%;top:6%;width:66%;height:88%;border:3px solid #00ef88;border-radius:10px;box-sizing:border-box;pointer-events:none}
.label{position:absolute;left:17%;top:6%;transform:translateY(-100%);background:rgba(0,0,0,.75);padding:3px 6px;color:#00ef88;font-size:11px}
.pred{font-size:27px;font-weight:bold;margin:8px 0}
.prow{display:grid;grid-template-columns:150px 1fr 65px;gap:8px;align-items:center;margin:8px 0}
.bar{height:12px;background:#333;border-radius:20px;overflow:hidden}.bar div{height:100%;background:#00b86b}
button{border:0;border-radius:8px;padding:9px 11px;margin:4px;background:#333;color:white}
.good{background:#14723a}.bad{background:#812b2b}
.small{opacity:.72;font-size:12px}
.warn{background:#4a3410;padding:9px;border-radius:8px;margin:8px 0}
pre{white-space:pre-wrap;background:#090909;padding:10px;border-radius:8px}
</style>
</head>
<body><div class="wrap">

<div class="card">
<h2>SafeSeat Camera ML Validation — V7 Collector-Parity Preview</h2>
<div class="warn">
Camera capture/settings now match the dataset collector. Preview is prioritized; ML is intentionally paced. Use ML Source Frame + Input MAD to verify the model is receiving fresh images.
</div>
<div class="cam">
<img id="cam" src="/frame">
<div class="guide"></div>
<div class="label">SUBJECT FRAMING GUIDE — not detector output</div>
</div>
<p class="small">The green rectangle is a framing guide only.</p>
</div>

<div class="card">
<div class="pred" id="vote">5-frame vote: building...</div>
<div id="vconf">Vote confidence: --</div>
<div id="pred">Latest frame: waiting...</div>
<div id="conf">Frame confidence: --</div>
<div id="inf">Inference: --</div>
<div id="inage">Prediction age: --</div>
<div id="fresh">Preview frame age: --</div>
<div id="health">Frames: --</div>
<div id="mlsrc">ML source frame: --</div>
<div id="mad">ML input MAD: --</div>
<div><a href="/mlframe" target="_blank" style="color:#00ef88">Open latest exact ML source JPEG</a></div>
<div id="exp">Expected: NOT SET</div>
<div id="rec">Recording: NO</div>
<hr>
<div class="prow"><span>leaning_backward</span><div class="bar"><div id="b0"></div></div><span id="v0">0%</span></div>
<div class="prow"><span>leaning_forward</span><div class="bar"><div id="b1"></div></div><span id="v1">0%</span></div>
<div class="prow"><span>leaning_left</span><div class="bar"><div id="b2"></div></div><span id="v2">0%</span></div>
<div class="prow"><span>leaning_right</span><div class="bar"><div id="b3"></div></div><span id="v3">0%</span></div>
<div class="prow"><span>upright</span><div class="bar"><div id="b4"></div></div><span id="v4">0%</span></div>
</div>

<div class="card">
<b>Expected posture</b><br>
<button onclick="cmd('u')">Upright</button>
<button onclick="cmd('f')">Forward</button>
<button onclick="cmd('l')">Left</button>
<button onclick="cmd('r')">Right</button>
<button onclick="cmd('b')">Backward</button>
<button onclick="cmd('x')">Clear</button><br><br>
<button class="good" onclick="cmd('g')">Start Recording</button>
<button class="bad" onclick="cmd('p')">Pause</button>
<button onclick="cmd('c')">Clear Counters</button>
<button onclick="summary()">Show Summary</button>
<pre id="sum">No summary requested yet.</pre>
</div>

</div>
<script>
async function cmd(c){await fetch('/cmd?c='+c);await status()}
async function summary(){document.getElementById('sum').textContent=await(await fetch('/summary?_='+Date.now())).text()}
async function status(){
 try{
  let s=await(await fetch('/status?_='+Date.now())).json();
  pred.textContent='Latest frame: '+s.prediction;
  conf.textContent='Frame confidence: '+(s.confidence*100).toFixed(2)+'%';
  vote.textContent='5-frame vote: '+s.voted_prediction+' ('+s.vote_progress+'/5 building, completed votes='+s.votes+')';
  vconf.textContent='Vote confidence: '+(s.voted_confidence*100).toFixed(2)+'%';
  inf.textContent='Inference duration: '+s.inference_ms+' ms';
  inage.textContent='Prediction age: '+s.prediction_age_ms+' ms';
  fresh.textContent='Preview frame age: '+s.frame_age_ms+' ms';
  health.textContent='Preview frames: '+s.frames+' | ML runs: '+s.inferences+
    ' | Capture fails: '+s.capture_failures+' | Decode fails: '+s.decode_failures;
  mlsrc.textContent='ML source frame: '+s.ml_source_frame+' | JPEG '+s.ml_jpeg_len+' B | hash 0x'+s.ml_jpeg_hash;
  mad.textContent='ML input MAD: '+s.input_mad.toFixed(2)+' INT8 levels';
  exp.textContent='Expected: '+s.expected;
  rec.textContent='Recording: '+(s.recording?'YES':'NO');
  s.probabilities.forEach((p,i)=>{
    let x=p*100;
    document.getElementById('b'+i).style.width=x+'%';
    document.getElementById('v'+i).textContent=x.toFixed(1)+'%';
  });
 }catch(e){}
}

// Collector-style preview with NO overlapping image requests.
// A new frame is requested only after the previous one loaded/failed.
const camEl=document.getElementById('cam');
function nextFrame(){
 setTimeout(()=>{camEl.src='/frame?_='+Date.now();},80);
}
camEl.onload=nextFrame;
camEl.onerror=nextFrame;
nextFrame();

// Status is deliberately slower so it cannot crowd the image endpoint.
setInterval(status,1000);
status();
</script>
</body></html>
)rawliteral";

// ============================================================
// Helpers
// ============================================================

static void addHttpHeaders()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "0");
}

static uint32_t sampledJpegHash(const uint8_t* data, size_t len)
{
    // Lightweight FNV-1a over a bounded sample of the JPEG.
    // It is a frame-freshness fingerprint, not a cryptographic hash.
    uint32_t h = 2166136261UL;

    if (!data || len == 0)
    {
        return h;
    }

    size_t step = len / 128U;
    if (step < 1U) step = 1U;

    for (size_t i = 0; i < len; i += step)
    {
        h ^= data[i];
        h *= 16777619UL;
    }

    h ^= (uint32_t)len;
    h *= 16777619UL;
    return h;
}

static int clampInt8(int v)
{
    if (v < -128) return -128;
    if (v > 127) return 127;
    return v;
}

static void resetVoteAccumulator()
{
    voteFrameCount = 0;

    for (int i = 0; i < CLASS_COUNT; ++i)
    {
        voteAccumulator[i] = 0.0f;
    }
}

static void setCommand(char c)
{
    if (xSemaphoreTake(resultMutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return;
    }

    switch (c)
    {
        case 'u': case 'U':
            expectedClass = UPRIGHT;
            recordSamples = false;
            resetVoteAccumulator();
            break;

        case 'f': case 'F':
            expectedClass = LEANING_FORWARD;
            recordSamples = false;
            resetVoteAccumulator();
            break;

        case 'l': case 'L':
            expectedClass = LEANING_LEFT;
            recordSamples = false;
            resetVoteAccumulator();
            break;

        case 'r': case 'R':
            expectedClass = LEANING_RIGHT;
            recordSamples = false;
            resetVoteAccumulator();
            break;

        case 'b': case 'B':
            expectedClass = LEANING_BACKWARD;
            recordSamples = false;
            resetVoteAccumulator();
            break;

        case 'g': case 'G':
            if (expectedClass >= 0)
            {
                recordSamples = true;
            }
            break;

        case 'p': case 'P':
            recordSamples = false;
            break;

        case 'c': case 'C':
            memset(confusion, 0, sizeof(confusion));
            voteCounter = 0;
            latestVotedPrediction = -1;
            latestVotedConfidence = 0.0f;
            resetVoteAccumulator();
            break;

        case 'x': case 'X':
            expectedClass = -1;
            recordSamples = false;
            resetVoteAccumulator();
            break;
    }

    xSemaphoreGive(resultMutex);
}

static String summaryText()
{
    String s;
    s.reserve(1100);

    if (xSemaphoreTake(resultMutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        return "Summary busy.";
    }

    uint32_t total = 0;
    uint32_t correct = 0;

    for (int a = 0; a < CLASS_COUNT; ++a)
    {
        for (int p = 0; p < CLASS_COUNT; ++p)
        {
            total += confusion[a][p];

            if (a == p)
            {
                correct += confusion[a][p];
            }
        }
    }

    s += "SAFESEAT LIVE CAMERA VALIDATION — 5-FRAME VOTE\n\n";
    s += "Completed 5-frame vote samples: " + String(total) + "\n";
    s += "Correct: " + String(correct) + "\n";

    if (total > 0)
    {
        s += "Accuracy: " + String(
            100.0f * (float)correct / (float)total,
            2
        ) + "%\n";
    }
    else
    {
        s += "Accuracy: N/A\n";
    }

    s += "\nEXP\\PRED\tBACK\tFWD\tLEFT\tRIGHT\tUP\n";

    for (int a = 0; a < CLASS_COUNT; ++a)
    {
        s += SHORT_NAMES[a];

        for (int p = 0; p < CLASS_COUNT; ++p)
        {
            s += "\t";
            s += String(confusion[a][p]);
        }

        s += "\n";
    }

    xSemaphoreGive(resultMutex);

    return s;
}

// ============================================================
// Camera initialization
// ============================================================

static bool initCamera()
{
    camera_config_t config = {};

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

    // JPEG is required here for stable Wi-Fi preview.
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;

    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK)
    {
        Serial.print("[CAMERA] init failed 0x");
        Serial.println(err, HEX);
        return false;
    }

    sensor_t* sensor = esp_camera_sensor_get();

    if (!sensor)
    {
        return false;
    }

    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);

    delay(350);

    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb)
    {
        return false;
    }

    bool ok =
        fb->format == PIXFORMAT_JPEG &&
        fb->width == 320 &&
        fb->height == 240 &&
        fb->len > 0;

    Serial.print("[CAMERA] JPEG self-test ");
    Serial.print(fb->width);
    Serial.print("x");
    Serial.print(fb->height);
    Serial.print(" bytes=");
    Serial.println(fb->len);

    esp_camera_fb_return(fb);

    return ok;
}

// ============================================================
// Model initialization
// ============================================================

static bool initModel()
{
    Serial.print("[ML] model bytes=");
    Serial.println(g_safeseat_posture_model_len);

    if (g_safeseat_posture_model_len != EXPECTED_MODEL_BYTES)
    {
        Serial.println("[ML] WARNING expected 117824 bytes.");
    }

    model = tflite::GetModel(g_safeseat_posture_model);

    if (!model)
    {
        return false;
    }

    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        Serial.println("[ML] schema mismatch.");
        return false;
    }

    if (resolver.AddPad() != kTfLiteOk) return false;
    if (resolver.AddConv2D() != kTfLiteOk) return false;
    if (resolver.AddMean() != kTfLiteOk) return false;
    if (resolver.AddFullyConnected() != kTfLiteOk) return false;

    tensorArena = static_cast<uint8_t*>(
        heap_caps_malloc(
            TENSOR_ARENA_BYTES,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    rgb888Buffer = static_cast<uint8_t*>(
        heap_caps_malloc(
            RGB888_BYTES,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    previewJpeg = static_cast<uint8_t*>(
        heap_caps_malloc(
            JPEG_BUFFER_CAPACITY,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    httpJpeg = static_cast<uint8_t*>(
        heap_caps_malloc(
            JPEG_BUFFER_CAPACITY,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    mlJpeg = static_cast<uint8_t*>(
        heap_caps_malloc(
            JPEG_BUFFER_CAPACITY,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    if (
        !tensorArena ||
        !rgb888Buffer ||
        !previewJpeg ||
        !httpJpeg ||
        !mlJpeg
    )
    {
        Serial.println("[ML] PSRAM allocation failed.");
        return false;
    }

    static tflite::MicroInterpreter staticInterpreter(
        model,
        resolver,
        tensorArena,
        TENSOR_ARENA_BYTES
    );

    interpreter = &staticInterpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk)
    {
        Serial.println("[ML] AllocateTensors failed.");
        return false;
    }

    inputTensor = interpreter->input(0);
    outputTensor = interpreter->output(0);

    if (!inputTensor || !outputTensor)
    {
        return false;
    }

    bool inputOK =
        inputTensor->type == kTfLiteInt8 &&
        inputTensor->dims->size == 4 &&
        inputTensor->dims->data[0] == 1 &&
        inputTensor->dims->data[1] == MODEL_H &&
        inputTensor->dims->data[2] == MODEL_W &&
        inputTensor->dims->data[3] == MODEL_C;

    int outputLast =
        outputTensor->dims->data[
            outputTensor->dims->size - 1
        ];

    bool outputOK =
        outputTensor->type == kTfLiteInt8 &&
        outputLast == CLASS_COUNT;

    if (!inputOK || !outputOK)
    {
        Serial.println("[ML] tensor contract mismatch.");
        return false;
    }

    Serial.print("[ML] input scale=");
    Serial.print(inputTensor->params.scale, 10);
    Serial.print(" zero=");
    Serial.println(inputTensor->params.zero_point);

    Serial.print("[ML] output scale=");
    Serial.print(outputTensor->params.scale, 10);
    Serial.print(" zero=");
    Serial.println(outputTensor->params.zero_point);

    Serial.print("[ML] free PSRAM=");
    Serial.println(ESP.getFreePsram());

    return true;
}

// ============================================================
// Camera task
// ============================================================

static void cameraTask(void* parameter)
{
    (void)parameter;

    unsigned long lastCapture = 0;

    for (;;)
    {
        unsigned long now = millis();

        if (now - lastCapture < CAMERA_CAPTURE_INTERVAL_MS)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        lastCapture = now;

        camera_fb_t* fb = esp_camera_fb_get();

        if (!fb)
        {
            captureFailures++;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        frameCounter++;
        latestFrameAt = millis();

        if (
            fb->format == PIXFORMAT_JPEG &&
            fb->len <= JPEG_BUFFER_CAPACITY
        )
        {
            // Always update browser preview.
            if (
                xSemaphoreTake(
                    previewMutex,
                    pdMS_TO_TICKS(30)
                )
                ==
                pdTRUE
            )
            {
                memcpy(previewJpeg, fb->buf, fb->len);
                previewJpegLen = fb->len;

                xSemaphoreGive(previewMutex);
            }

            // Only queue a new ML snapshot when ML task has consumed the old one.
            if (
                !mlSnapshotPending
                &&
                !mlInferenceBusy
            )
            {
                if (
                    xSemaphoreTake(
                        mlJpegMutex,
                        pdMS_TO_TICKS(30)
                    )
                    ==
                    pdTRUE
                )
                {
                    memcpy(mlJpeg, fb->buf, fb->len);
                    mlJpegLen = fb->len;
                    mlSnapshotFrameId = frameCounter;
                    mlSnapshotPending = true;

                    xSemaphoreGive(mlJpegMutex);
                }
            }
        }

        esp_camera_fb_return(fb);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================
// ML preprocessing
// ============================================================

static int8_t quantizePixel(uint8_t pixel)
{
    // NEW model contract:
    // real input = RGB / 255.0
    //
    // Current validated INT8 tensor parameters are:
    //   scale = 1/255 ~= 0.0039215689
    //   zero  = -128
    //
    // Keep the generic quantization formula so runtime still checks
    // the tensor contract rather than hard-coding pixel-128.
    float realValue = (float)pixel / 255.0f;
    float scale = inputTensor->params.scale;
    int zero = inputTensor->params.zero_point;

    int q =
        (int)lroundf(realValue / scale)
        +
        zero;

    return (int8_t)clampInt8(q);
}

static bool decodeSnapshotToRgb()
{
    if (
        xSemaphoreTake(
            mlJpegMutex,
            pdMS_TO_TICKS(500)
        )
        !=
        pdTRUE
    )
    {
        return false;
    }

    if (mlJpegLen == 0)
    {
        xSemaphoreGive(mlJpegMutex);
        return false;
    }

    bool ok = fmt2rgb888(
        mlJpeg,
        mlJpegLen,
        PIXFORMAT_JPEG,
        rgb888Buffer
    );

    // Keep mlSnapshotPending asserted while inference is running.
    // The ML task clears it only after the current Invoke() finishes,
    // preventing a stale "next" frame from being queued ~20 s early.
    xSemaphoreGive(mlJpegMutex);

    return ok;
}

static bool fillInputFromRgb888()
{
    int8_t* dst = inputTensor->data.int8;

    if (!dst)
    {
        return false;
    }

    // Training/evaluation used the full 320x240 camera frame resized to
    // 64x64. Because 320/64 = 5 exactly and 240/64 = 3.75, this
    // implements an area-style box average:
    //
    //   horizontal: exactly 5 source pixels per output pixel
    //   vertical  : exact 3.75-row coverage using quarter-row weights
    //
    // This is substantially closer to cv2 INTER_AREA than nearest-neighbor
    // resize and is inexpensive at only 4096 output pixels.
    size_t out = 0;

    for (int y = 0; y < MODEL_H; ++y)
    {
        // Source Y interval in quarter-pixel units:
        // y * 240/64 == y * 15/4.
        int yStart4 = y * 15;
        int yEnd4 = (y + 1) * 15;

        int syFirst = yStart4 / 4;
        int syLastExclusive = (yEnd4 + 3) / 4;

        for (int x = 0; x < MODEL_W; ++x)
        {
            int sx0 = x * 5;

            uint32_t sumR = 0;
            uint32_t sumG = 0;
            uint32_t sumB = 0;
            uint32_t totalWeight = 0;

            for (int sy = syFirst; sy < syLastExclusive; ++sy)
            {
                if (sy < 0 || sy >= 240)
                {
                    continue;
                }

                int rowStart4 = sy * 4;
                int rowEnd4 = (sy + 1) * 4;

                int overlapStart =
                    yStart4 > rowStart4 ? yStart4 : rowStart4;
                int overlapEnd =
                    yEnd4 < rowEnd4 ? yEnd4 : rowEnd4;

                int rowWeight = overlapEnd - overlapStart;

                if (rowWeight <= 0)
                {
                    continue;
                }

                for (int sx = sx0; sx < sx0 + 5; ++sx)
                {
                    size_t src =
                        ((size_t)sy * 320U + (size_t)sx) * 3U;

                    // Espressif fmt2rgb888() emits B,G,R byte order.
                    uint8_t b = rgb888Buffer[src + 0];
                    uint8_t g = rgb888Buffer[src + 1];
                    uint8_t r = rgb888Buffer[src + 2];

                    sumR += (uint32_t)r * (uint32_t)rowWeight;
                    sumG += (uint32_t)g * (uint32_t)rowWeight;
                    sumB += (uint32_t)b * (uint32_t)rowWeight;
                    totalWeight += (uint32_t)rowWeight;
                }
            }

            if (totalWeight == 0)
            {
                return false;
            }

            uint8_t r =
                (uint8_t)((sumR + totalWeight / 2U) / totalWeight);
            uint8_t g =
                (uint8_t)((sumG + totalWeight / 2U) / totalWeight);
            uint8_t b =
                (uint8_t)((sumB + totalWeight / 2U) / totalWeight);

            dst[out++] = quantizePixel(r);
            dst[out++] = quantizePixel(g);
            dst[out++] = quantizePixel(b);
        }
    }

    return out == (size_t)(MODEL_W * MODEL_H * MODEL_C);
}

static bool decodeModelOutput(
    int& prediction,
    float& confidence,
    float probabilities[CLASS_COUNT]
)
{
    const int8_t* q = outputTensor->data.int8;

    if (!q)
    {
        return false;
    }

    float logits[CLASS_COUNT];
    float maxLogit = -INFINITY;

    for (int i = 0; i < CLASS_COUNT; ++i)
    {
        logits[i] =
            ((int)q[i] - outputTensor->params.zero_point)
            *
            outputTensor->params.scale;

        if (logits[i] > maxLogit)
        {
            maxLogit = logits[i];
        }
    }

    float denom = 0.0f;

    for (int i = 0; i < CLASS_COUNT; ++i)
    {
        probabilities[i] =
            expf(logits[i] - maxLogit);

        denom += probabilities[i];
    }

    if (!isfinite(denom) || denom <= 0.0f)
    {
        return false;
    }

    prediction = 0;
    confidence = -1.0f;

    for (int i = 0; i < CLASS_COUNT; ++i)
    {
        probabilities[i] /= denom;

        if (probabilities[i] > confidence)
        {
            confidence = probabilities[i];
            prediction = i;
        }
    }

    return true;
}

// ============================================================
// ML task
// ============================================================

static void mlTask(void* parameter)
{
    (void)parameter;

    for (;;)
    {
        if (!mlSnapshotPending)
        {
            vTaskDelay(pdMS_TO_TICKS(25));
            continue;
        }

        mlInferenceBusy = true;

        if (!decodeSnapshotToRgb())
        {
            decodeFailures++;
            mlSnapshotPending = false;
            mlInferenceBusy = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!fillInputFromRgb888())
        {
            decodeFailures++;
            mlSnapshotPending = false;
            mlInferenceBusy = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Verify the ACTUAL model tensor is changing between inferences.
        // This directly answers whether ML is seeing a frozen image.
        {
            const int8_t* cur = inputTensor->data.int8;
            const size_t n = (size_t)(MODEL_W * MODEL_H * MODEL_C);

            if (cur && previousModelInputValid)
            {
                uint64_t sumAbs = 0;

                for (size_t i = 0; i < n; ++i)
                {
                    int d = (int)cur[i] - (int)previousModelInput[i];
                    if (d < 0) d = -d;
                    sumAbs += (uint32_t)d;
                }

                latestInputMAD = (float)sumAbs / (float)n;
            }
            else
            {
                latestInputMAD = 0.0f;
            }

            if (cur)
            {
                memcpy(previousModelInput, cur, n);
                previousModelInputValid = true;
            }
        }

        // Capture exact source identity before Invoke().
        latestInferenceSourceFrameId = mlSnapshotFrameId;
        latestInferenceJpegLen = mlJpegLen;
        latestInferenceJpegHash = sampledJpegHash(mlJpeg, mlJpegLen);

        unsigned long start = millis();

        TfLiteStatus status =
            interpreter->Invoke();

        unsigned long duration =
            millis() - start;

        if (status != kTfLiteOk)
        {
            Serial.println("[ML] Invoke failed.");
            mlSnapshotPending = false;
            mlInferenceBusy = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int prediction = -1;
        float confidence = 0.0f;
        float probs[CLASS_COUNT] = {0};

        if (
            !decodeModelOutput(
                prediction,
                confidence,
                probs
            )
        )
        {
            decodeFailures++;
            mlSnapshotPending = false;
            mlInferenceBusy = false;
            continue;
        }

        if (
            xSemaphoreTake(
                resultMutex,
                pdMS_TO_TICKS(500)
            )
            ==
            pdTRUE
        )
        {
            latestPrediction = prediction;
            latestConfidence = confidence;
            latestInferenceMs = duration;
            latestInferenceAt = millis();

            for (int i = 0; i < CLASS_COUNT; ++i)
            {
                latestProbabilities[i] = probs[i];
            }

            inferenceCounter++;

            // Accumulate five consecutive model probability vectors.
            for (int i = 0; i < CLASS_COUNT; ++i)
            {
                voteAccumulator[i] += probs[i];
            }

            voteFrameCount++;

            if (voteFrameCount >= VOTE_FRAMES)
            {
                int votedPrediction = 0;
                float votedConfidence = -1.0f;

                for (int i = 0; i < CLASS_COUNT; ++i)
                {
                    float avg =
                        voteAccumulator[i] / (float)VOTE_FRAMES;

                    latestVotedProbabilities[i] = avg;

                    if (avg > votedConfidence)
                    {
                        votedConfidence = avg;
                        votedPrediction = i;
                    }
                }

                latestVotedPrediction = votedPrediction;
                latestVotedConfidence = votedConfidence;
                voteCounter++;

                // Formal live counters use the exact 5-frame deployment vote,
                // not individual noisy frame predictions.
                if (
                    recordSamples &&
                    expectedClass >= 0
                )
                {
                    confusion[expectedClass][votedPrediction]++;
                }

                resetVoteAccumulator();
            }

            xSemaphoreGive(resultMutex);
        }

        Serial.println();
        Serial.print("[CAM-ML] previewFrame=");
        Serial.print(frameCounter);
        Serial.print(" srcFrame=");
        Serial.print(latestInferenceSourceFrameId);
        Serial.print(" jpeg=");
        Serial.print(latestInferenceJpegLen);
        Serial.print(" hash=0x");
        Serial.print(latestInferenceJpegHash, HEX);
        Serial.print(" inputMAD=");
        Serial.print(latestInputMAD, 2);
        Serial.print(" mlRun=");
        Serial.print(inferenceCounter);
        Serial.print(" pred=");
        Serial.print(CLASS_NAMES[prediction]);
        Serial.print(" conf=");
        Serial.print(confidence * 100.0f, 1);
        Serial.print("% infer=");
        Serial.print(duration);
        Serial.print("ms");

        if (latestVotedPrediction >= 0)
        {
            Serial.print(" vote5=");
            Serial.print(CLASS_NAMES[latestVotedPrediction]);
            Serial.print(" voteConf=");
            Serial.print(latestVotedConfidence * 100.0f, 1);
            Serial.print("% votes=");
            Serial.print(voteCounter);
        }
        else
        {
            Serial.print(" vote5=BUILDING(");
            Serial.print(voteFrameCount);
            Serial.print("/5)");
        }

        Serial.print(" freePSRAM=");
        Serial.println(ESP.getFreePsram());

        // Current inference is complete. Allow camera task to queue
        // the newest available frame for the NEXT inference.
        mlSnapshotPending = false;
        mlInferenceBusy = false;

        vTaskDelay(pdMS_TO_TICKS(ML_MIN_GAP_MS));
    }
}

// ============================================================
// HTTP
// ============================================================

static void handleRoot()
{
    addHttpHeaders();
    server.send_P(
        200,
        "text/html",
        PAGE
    );
}

static void handleFrame()
{
    if (
        xSemaphoreTake(
            previewMutex,
            pdMS_TO_TICKS(50)
        )
        !=
        pdTRUE
    )
    {
        addHttpHeaders();
        server.send(503, "text/plain", "Preview busy");
        return;
    }

    if (previewJpegLen == 0)
    {
        xSemaphoreGive(previewMutex);
        addHttpHeaders();
        server.send(503, "text/plain", "No frame yet");
        return;
    }

    // Copy quickly, then release previewMutex BEFORE Wi-Fi transmission.
    httpJpegLen = previewJpegLen;
    memcpy(httpJpeg, previewJpeg, previewJpegLen);
    xSemaphoreGive(previewMutex);

    addHttpHeaders();
    server.setContentLength(httpJpegLen);
    server.send(200, "image/jpeg", "");

    WiFiClient client = server.client();
    client.write(httpJpeg, httpJpegLen);
}

static void handleMlFrame()
{
    if (
        xSemaphoreTake(
            mlJpegMutex,
            pdMS_TO_TICKS(100)
        )
        !=
        pdTRUE
    )
    {
        addHttpHeaders();
        server.send(503, "text/plain", "ML frame busy");
        return;
    }

    if (mlJpegLen == 0)
    {
        xSemaphoreGive(mlJpegMutex);
        addHttpHeaders();
        server.send(503, "text/plain", "No ML frame yet");
        return;
    }

    httpJpegLen = mlJpegLen;
    memcpy(httpJpeg, mlJpeg, mlJpegLen);
    xSemaphoreGive(mlJpegMutex);

    addHttpHeaders();
    server.setContentLength(httpJpegLen);
    server.send(200, "image/jpeg", "");

    WiFiClient client = server.client();
    client.write(httpJpeg, httpJpegLen);
}

static void handleStatus()
{
    unsigned long frameAge =
        latestFrameAt == 0
            ? 0
            : millis() - latestFrameAt;

    unsigned long predictionAge =
        latestInferenceAt == 0
            ? 0
            : millis() - latestInferenceAt;

    int prediction;
    float confidence;
    float probs[CLASS_COUNT];
    int votedPrediction;
    float votedConfidence;
    int voteProgress;
    uint32_t votes;
    unsigned long inferenceMs;
    int expected;
    bool recording;

    if (
        xSemaphoreTake(
            resultMutex,
            pdMS_TO_TICKS(100)
        )
        ==
        pdTRUE
    )
    {
        prediction = latestPrediction;
        confidence = latestConfidence;
        inferenceMs = latestInferenceMs;
        expected = expectedClass;
        recording = recordSamples;
        votedPrediction = latestVotedPrediction;
        votedConfidence = latestVotedConfidence;
        voteProgress = voteFrameCount;
        votes = voteCounter;

        for (int i = 0; i < CLASS_COUNT; ++i)
        {
            probs[i] = latestProbabilities[i];
        }

        xSemaphoreGive(resultMutex);
    }
    else
    {
        prediction = latestPrediction;
        confidence = latestConfidence;
        inferenceMs = latestInferenceMs;
        expected = expectedClass;
        recording = recordSamples;
        votedPrediction = latestVotedPrediction;
        votedConfidence = latestVotedConfidence;
        voteProgress = voteFrameCount;
        votes = voteCounter;

        for (int i = 0; i < CLASS_COUNT; ++i)
        {
            probs[i] = latestProbabilities[i];
        }
    }

    String j;
    j.reserve(900);

    j += "{";

    j += "\"prediction\":\"";
    j += prediction >= 0
        ? CLASS_NAMES[prediction]
        : "waiting";
    j += "\",";

    j += "\"confidence\":";
    j += String(
        confidence < 0 ? 0 : confidence,
        6
    );
    j += ",";

    j += "\"voted_prediction\":\"";
    j += votedPrediction >= 0
        ? CLASS_NAMES[votedPrediction]
        : "building";
    j += "\",";

    j += "\"voted_confidence\":";
    j += String(
        votedConfidence < 0 ? 0 : votedConfidence,
        6
    );
    j += ",";

    j += "\"vote_progress\":";
    j += String(voteProgress);
    j += ",";

    j += "\"votes\":";
    j += String(votes);
    j += ",";

    j += "\"inference_ms\":";
    j += String(inferenceMs);
    j += ",";

    j += "\"prediction_age_ms\":";
    j += String(predictionAge);
    j += ",";

    j += "\"frame_age_ms\":";
    j += String(frameAge);
    j += ",";

    j += "\"frames\":";
    j += String(frameCounter);
    j += ",";

    j += "\"inferences\":";
    j += String(inferenceCounter);
    j += ",";

    j += "\"capture_failures\":";
    j += String(captureFailures);
    j += ",";

    j += "\"decode_failures\":";
    j += String(decodeFailures);
    j += ",";

    j += "\"ml_source_frame\":";
    j += String(latestInferenceSourceFrameId);
    j += ",";

    j += "\"ml_jpeg_len\":";
    j += String((uint32_t)latestInferenceJpegLen);
    j += ",";

    char hashBuf[9];
    snprintf(hashBuf, sizeof(hashBuf), "%08lX", (unsigned long)latestInferenceJpegHash);
    j += "\"ml_jpeg_hash\":\"";
    j += hashBuf;
    j += "\",";

    j += "\"input_mad\":";
    j += String(latestInputMAD, 3);
    j += ",";

    j += "\"expected\":\"";
    j += expected >= 0
        ? CLASS_NAMES[expected]
        : "NOT SET";
    j += "\",";

    j += "\"recording\":";
    j += recording ? "true" : "false";
    j += ",";

    j += "\"probabilities\":[";

    for (int i = 0; i < CLASS_COUNT; ++i)
    {
        if (i)
        {
            j += ",";
        }

        j += String(probs[i], 6);
    }

    j += "]";
    j += "}";

    addHttpHeaders();
    server.send(
        200,
        "application/json",
        j
    );
}

static void handleCmd()
{
    if (
        server.hasArg("c") &&
        server.arg("c").length()
    )
    {
        setCommand(
            server.arg("c").charAt(0)
        );
    }

    addHttpHeaders();
    server.send(
        200,
        "text/plain",
        "OK"
    );
}

static void startServer()
{
    server.on("/", handleRoot);

    server.on(
        "/frame",
        handleFrame
    );

    server.on(
        "/status",
        handleStatus
    );

    server.on(
        "/mlframe",
        handleMlFrame
    );

    server.on(
        "/summary",
        []()
        {
            server.send(
                200,
                "text/plain",
                summaryText()
            );
        }
    );

    server.on(
        "/cmd",
        handleCmd
    );

    server.begin();
}

// ============================================================
// Serial
// ============================================================

static void serialControls()
{
    while (Serial.available())
    {
        char c =
            (char)Serial.read();

        if (c == 's' || c == 'S')
        {
            Serial.println(
                summaryText()
            );
        }
        else
        {
            setCommand(c);
        }
    }
}

// ============================================================
// Task-watchdog configuration
//
// The previous 160x160 SafeSeat posture CNN took roughly 19.7 seconds for
// one TFLite Micro Invoke() on this ESP32-S3 build.
//
// Default TWDT settings are much shorter, so CPU0's IDLE task was
// starved during legitimate inference and the board rebooted with:
//
//   task_wdt: IDLE0 ... CPU 0: SafeSeatMLTask
//
// We keep watchdog protection enabled, but extend the timeout to
// 45 seconds. That is comfortably above the observed ~20 s model
// latency while still detecting a genuine long hang.
// ============================================================

static bool configureTaskWatchdogForLongInference()
{
    esp_task_wdt_config_t config = {};

    config.timeout_ms = 45000;

    // Keep both idle tasks monitored.
    config.idle_core_mask =
        (1U << 0) |
        (1U << 1);

    config.trigger_panic = true;

    esp_err_t result =
        esp_task_wdt_reconfigure(
            &config
        );

    Serial.print(
        "[WDT] Reconfigure 45 s: "
    );

    if (result == ESP_OK)
    {
        Serial.println("PASS");
        return true;
    }

    Serial.print(
        "FAILED, code="
    );
    Serial.println(
        static_cast<int>(result)
    );

    return false;
}


// ============================================================
// Setup / loop
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1800);

    Serial.println();
    Serial.println("================================================");
    Serial.println(" SAFESEAT CAMERA ML V7 COLLECTOR-PARITY");
    Serial.println(" collector-speed preview + paced 64x64 INT8 ML");
    Serial.println("================================================");

    // Configure TWDT before the long-running ML task starts.
    // If this fails we still continue and print the error, but the
    // board may reboot during ~20 s inference under a short default
    // watchdog timeout.
    configureTaskWatchdogForLongInference();

    if (!psramFound())
    {
        Serial.println(
            "FATAL: PSRAM not detected."
        );
        return;
    }

    previewMutex =
        xSemaphoreCreateMutex();

    mlJpegMutex =
        xSemaphoreCreateMutex();

    resultMutex =
        xSemaphoreCreateMutex();

    if (
        !previewMutex ||
        !mlJpegMutex ||
        !resultMutex
    )
    {
        Serial.println(
            "FATAL: mutex creation failed."
        );
        return;
    }

    cameraReady =
        initCamera();

    if (!cameraReady)
    {
        Serial.println(
            "FATAL: camera initialization failed."
        );
        return;
    }

    modelReady =
        initModel();

    if (!modelReady)
    {
        Serial.println(
            "FATAL: model initialization failed."
        );
        return;
    }

    WiFi.mode(WIFI_AP);

    if (
        !WiFi.softAP(
            AP_SSID,
            AP_PASSWORD
        )
    )
    {
        Serial.println(
            "FATAL: Wi-Fi AP failed."
        );
        return;
    }

    Serial.println();
    Serial.println("[Wi-Fi] READY");
    Serial.print("[Wi-Fi] SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[Wi-Fi] PASS: ");
    Serial.println(AP_PASSWORD);
    Serial.print("[Wi-Fi] URL : http://");
    Serial.println(WiFi.softAPIP());

    startServer();

    // Camera preview task on core 1.
    BaseType_t camCreated =
        xTaskCreatePinnedToCore(
            cameraTask,
            "SafeSeatCameraTask",
            8192,
            nullptr,
            2,
            &cameraTaskHandle,
            1
        );

    // Match collection behavior more closely: allow camera AE/AWB and
    // browser preview to settle before beginning ML.
    delay(2000);

    // ML task on the other core, intentionally paced for Wi-Fi responsiveness.
    BaseType_t mlCreated =
        xTaskCreatePinnedToCore(
            mlTask,
            "SafeSeatMLTask",
            12288,
            nullptr,
            1,
            &mlTaskHandle,
            0
        );

    if (
        camCreated != pdPASS ||
        mlCreated != pdPASS
    )
    {
        Serial.println(
            "FATAL: FreeRTOS task creation failed."
        );
        return;
    }

    Serial.println();
    Serial.println(
        "[V7] Camera task: Core 1 | collector settings"
    );
    Serial.println(
        "[V7] TFLite ML task: Core 0 | paced for Wi-Fi"
    );
    Serial.println(
        "[V7] Capture: 100 ms | JPEG quality 12 | latest-frame mode."
    );
    Serial.println(
        "[V7] Preprocess: BGR decode -> RGB -> exact area 64x64 -> RGB/255 INT8."
    );
    Serial.println(
        "[V7] ML log includes srcFrame/hash/inputMAD to prove freshness."
    );
}

void loop()
{
    server.handleClient();
    serialControls();

    delay(2);
}
