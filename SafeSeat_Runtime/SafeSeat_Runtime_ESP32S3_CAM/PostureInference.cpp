#include "PostureInference.h"

#include <math.h>
#include <new>

#include "Config.h"
#include "PostureModelData.h"
#include "img_converters.h"
#include "esp_heap_caps.h"

#if __has_include("tensorflow/lite/micro/micro_interpreter.h") && \
    __has_include("tensorflow/lite/micro/micro_mutable_op_resolver.h")
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#else
#error "TensorFlow Lite Micro headers not found. Install a compatible TFLM Arduino library before compiling SafeSeat_Runtime_ESP32_CAM."
#endif

namespace
{
    tflite::MicroMutableOpResolver<4> gResolver;
    bool gResolverConfigured = false;

    float clamp01(float x)
    {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }
}

bool PostureInference::begin()
{
    if (initialized)
    {
        return true;
    }

    psramAvailable = psramFound();
    if (!psramAvailable)
    {
        Serial.println("[CAM-ML] ERROR: PSRAM not detected. INT8 ESP32-S3 camera inference requires PSRAM.");
        return false;
    }

    tensorArena = static_cast<uint8_t *>(
        heap_caps_malloc(
            CAMERA_TENSOR_ARENA_BYTES,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    rgbBuffer = static_cast<uint8_t *>(
        heap_caps_malloc(
            CAMERA_RGB_BUFFER_BYTES,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    if (tensorArena == nullptr || rgbBuffer == nullptr)
    {
        Serial.println("[CAM-ML] ERROR: PSRAM allocation failed.");
        return false;
    }

    initialized = initializeInterpreter();
    return initialized;
}

bool PostureInference::initializeInterpreter()
{
    const tflite::Model *model = tflite::GetModel(g_posture_model);
    if (model == nullptr || model->version() != TFLITE_SCHEMA_VERSION)
    {
        Serial.println("[CAM-ML] ERROR: TFLite schema mismatch.");
        return false;
    }

    if (!gResolverConfigured)
    {
        if (gResolver.AddConv2D() != kTfLiteOk
            || gResolver.AddMaxPool2D() != kTfLiteOk
            || gResolver.AddMean() != kTfLiteOk
            || gResolver.AddFullyConnected() != kTfLiteOk)
        {
            Serial.println("[CAM-ML] ERROR: failed to register required TFLM operators.");
            return false;
        }
        gResolverConfigured = true;
    }

    auto *interpreter = new tflite::MicroInterpreter(
        model,
        gResolver,
        tensorArena,
        CAMERA_TENSOR_ARENA_BYTES
    );

    if (interpreter == nullptr)
    {
        Serial.println("[CAM-ML] ERROR: interpreter allocation failed.");
        return false;
    }

    if (interpreter->AllocateTensors() != kTfLiteOk)
    {
        Serial.println("[CAM-ML] ERROR: AllocateTensors failed. Increase PSRAM arena if needed.");
        return false;
    }

    TfLiteTensor *input = interpreter->input(0);
    TfLiteTensor *output = interpreter->output(0);

    if (input == nullptr || output == nullptr)
    {
        Serial.println("[CAM-ML] ERROR: model tensors unavailable.");
        return false;
    }

    if (input->type != kTfLiteInt8 || output->type != kTfLiteInt8)
    {
        Serial.println("[CAM-ML] ERROR: expected full INT8 input/output tensors.");
        return false;
    }

    if (input->dims == nullptr
        || input->dims->size != 4
        || input->dims->data[0] != 1
        || input->dims->data[1] != CAMERA_MODEL_INPUT_HEIGHT
        || input->dims->data[2] != CAMERA_MODEL_INPUT_WIDTH
        || input->dims->data[3] != CAMERA_MODEL_INPUT_CHANNELS)
    {
        Serial.println("[CAM-ML] ERROR: unexpected model input shape.");
        return false;
    }

    if (output->dims == nullptr
        || output->dims->data[output->dims->size - 1] != CAMERA_MODEL_CLASS_COUNT)
    {
        Serial.println("[CAM-ML] ERROR: unexpected model output shape.");
        return false;
    }

    if (!(input->params.scale > 0.0f) || !(output->params.scale > 0.0f))
    {
        Serial.println("[CAM-ML] ERROR: invalid quantization parameters.");
        return false;
    }

    // Exact runtime equivalent of training normalization:
    // RGB uint8 -> [0,1] -> (x-0.5)/0.5 -> INT8 quantization.
    for (int pixel = 0; pixel < 256; ++pixel)
    {
        const float normalized =
            (static_cast<float>(pixel) / 255.0f - 0.5f) / 0.5f;

        long quantized = lroundf(
            normalized / input->params.scale
        ) + input->params.zero_point;

        if (quantized < -128) quantized = -128;
        if (quantized > 127) quantized = 127;
        quantizeLut[pixel] = static_cast<int8_t>(quantized);
    }

    interpreterOpaque = interpreter;
    inputTensorOpaque = input;
    outputTensorOpaque = output;

    Serial.print("[CAM-ML] Model ready. Bytes: ");
    Serial.print(g_posture_model_len);
    Serial.print(" | Tensor arena: ");
    Serial.print(CAMERA_TENSOR_ARENA_BYTES / 1024);
    Serial.println(" KiB PSRAM");

    return true;
}

bool PostureInference::prepareInput(camera_fb_t *frame)
{
    if (frame == nullptr || frame->buf == nullptr)
    {
        return false;
    }

    if (!fmt2rgb888(
            frame->buf,
            frame->len,
            frame->format,
            rgbBuffer
        ))
    {
        return false;
    }

    auto *input = static_cast<TfLiteTensor *>(inputTensorOpaque);
    if (input == nullptr || input->data.int8 == nullptr)
    {
        return false;
    }

    const int srcW = frame->width;
    const int srcH = frame->height;

    if (srcW <= 0 || srcH <= 0
        || srcW > CAMERA_RGB_BUFFER_MAX_WIDTH
        || srcH > CAMERA_RGB_BUFFER_MAX_HEIGHT)
    {
        Serial.println("[CAM-ML] ERROR: capture dimensions exceed RGB buffer safety limit.");
        return false;
    }

    // Nearest-neighbor resize/stretch to 160x160. This mirrors the
    // training transform's direct square resize instead of applying
    // an untrained crop policy.
    for (int y = 0; y < CAMERA_MODEL_INPUT_HEIGHT; ++y)
    {
        int srcY = (y * srcH) / CAMERA_MODEL_INPUT_HEIGHT;
        if (srcY >= srcH) srcY = srcH - 1;

        for (int x = 0; x < CAMERA_MODEL_INPUT_WIDTH; ++x)
        {
            int srcX = (x * srcW) / CAMERA_MODEL_INPUT_WIDTH;
            if (srcX >= srcW) srcX = srcW - 1;

            const size_t srcIndex =
                (static_cast<size_t>(srcY) * srcW + srcX) * 3u;
            const size_t dstIndex =
                (static_cast<size_t>(y) * CAMERA_MODEL_INPUT_WIDTH + x) * 3u;

            input->data.int8[dstIndex + 0] = quantizeLut[rgbBuffer[srcIndex + 0]];
            input->data.int8[dstIndex + 1] = quantizeLut[rgbBuffer[srcIndex + 1]];
            input->data.int8[dstIndex + 2] = quantizeLut[rgbBuffer[srcIndex + 2]];
        }
    }

    return true;
}

PostureInferenceResult PostureInference::infer(camera_fb_t *frame)
{
    PostureInferenceResult result;

    if (!initialized || !prepareInput(frame))
    {
        return result;
    }

    auto *interpreter = static_cast<tflite::MicroInterpreter *>(interpreterOpaque);
    auto *output = static_cast<TfLiteTensor *>(outputTensorOpaque);

    const unsigned long start = millis();
    if (interpreter->Invoke() != kTfLiteOk)
    {
        return result;
    }
    result.inferenceMillis = millis() - start;

    float logits[CAMERA_MODEL_CLASS_COUNT];
    float maxLogit = -INFINITY;
    int bestIndex = 0;

    for (int i = 0; i < CAMERA_MODEL_CLASS_COUNT; ++i)
    {
        logits[i] =
            (static_cast<int>(output->data.int8[i]) - output->params.zero_point)
            * output->params.scale;

        if (logits[i] > maxLogit)
        {
            maxLogit = logits[i];
            bestIndex = i;
        }
    }

    float denominator = 0.0f;
    for (int i = 0; i < CAMERA_MODEL_CLASS_COUNT; ++i)
    {
        denominator += expf(logits[i] - maxLogit);
    }

    float confidence = 0.0f;
    if (denominator > 0.0f && isfinite(denominator))
    {
        confidence = 1.0f / denominator;
    }

    result.valid = true;
    result.posture = static_cast<CameraPostureClass>(bestIndex);
    result.confidence = clamp01(confidence);
    return result;
}
