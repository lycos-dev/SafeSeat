#include "SafeSeatRecovery.h"

#include <math.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
    static constexpr uint32_t RETAINED_BASELINE_MAGIC = 0x53465352UL; // "SFSR"
    static constexpr uint32_t RETAINED_BASELINE_VERSION = 1UL;
    static constexpr uint32_t SUPERVISOR_RESTART_MAGIC = 0x53575231UL; // "SWR1"

    struct RetainedFsrBaseline
    {
        uint32_t magic;
        uint32_t version;
        uint32_t count;
        float electricalBaseline[SAFESEAT_RETAINED_FSR_COUNT];
        uint32_t checksum;
    };

    // NOINIT is deliberate: these bytes must not be reinitialized by the C++
    // startup path after a software restart. Magic + checksum + reset-reason
    // checks reject random/stale RTC contents on a cold boot.
    RTC_NOINIT_ATTR RetainedFsrBaseline retainedFsrBaseline;
    RTC_NOINIT_ATTR uint32_t supervisorRestartMarker;

    volatile uint32_t lastLoopHeartbeatMillis = 0;
    volatile uint32_t configuredFreezeTimeoutMs = 0;
    TaskHandle_t supervisorTaskHandle = nullptr;

    uint32_t fnv1aUpdate(
        uint32_t hash,
        const uint8_t *data,
        size_t length
    )
    {
        for (size_t i = 0; i < length; ++i)
        {
            hash ^= data[i];
            hash *= 16777619UL;
        }

        return hash;
    }

    uint32_t retainedChecksum(const RetainedFsrBaseline &state)
    {
        uint32_t hash = 2166136261UL;

        hash = fnv1aUpdate(
            hash,
            reinterpret_cast<const uint8_t*>(&state.version),
            sizeof(state.version)
        );

        hash = fnv1aUpdate(
            hash,
            reinterpret_cast<const uint8_t*>(&state.count),
            sizeof(state.count)
        );

        hash = fnv1aUpdate(
            hash,
            reinterpret_cast<const uint8_t*>(state.electricalBaseline),
            sizeof(state.electricalBaseline)
        );

        return hash;
    }

    bool retainedBaselineLooksStructurallyValid()
    {
        if (
            retainedFsrBaseline.magic != RETAINED_BASELINE_MAGIC
            || retainedFsrBaseline.version != RETAINED_BASELINE_VERSION
            || retainedFsrBaseline.count != SAFESEAT_RETAINED_FSR_COUNT
            || retainedFsrBaseline.checksum != retainedChecksum(retainedFsrBaseline)
        )
        {
            return false;
        }

        for (size_t i = 0; i < SAFESEAT_RETAINED_FSR_COUNT; ++i)
        {
            const float value = retainedFsrBaseline.electricalBaseline[i];

            if (!isfinite(value) || value < 0.0f || value > 40000.0f)
            {
                return false;
            }
        }

        return true;
    }

    void supervisorTask(void *)
    {
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(250));

            const uint32_t timeoutMs = configuredFreezeTimeoutMs;
            const uint32_t lastBeat = lastLoopHeartbeatMillis;

            if (timeoutMs == 0 || lastBeat == 0)
            {
                continue;
            }

            const uint32_t now = millis();
            const uint32_t stalledFor = now - lastBeat;

            if (stalledFor < timeoutMs)
            {
                continue;
            }

            // Do not depend on Serial here. If the main task is stalled in
            // I/O, another blocking print would make recovery less reliable.
            supervisorRestartMarker = SUPERVISOR_RESTART_MAGIC;

            // Give RTC writes a scheduling point, then perform a controlled
            // software restart. The next boot can safely identify this path.
            vTaskDelay(pdMS_TO_TICKS(10));
            esp_restart();
        }
    }
}


bool safeSeatRecoverySaveFsrBaseline(
    const float *electricalBaseline,
    size_t count
)
{
    if (
        electricalBaseline == nullptr
        || count != SAFESEAT_RETAINED_FSR_COUNT
    )
    {
        return false;
    }

    RetainedFsrBaseline next;
    next.magic = RETAINED_BASELINE_MAGIC;
    next.version = RETAINED_BASELINE_VERSION;
    next.count = SAFESEAT_RETAINED_FSR_COUNT;

    for (size_t i = 0; i < SAFESEAT_RETAINED_FSR_COUNT; ++i)
    {
        const float value = electricalBaseline[i];

        if (!isfinite(value) || value < 0.0f || value > 40000.0f)
        {
            return false;
        }

        next.electricalBaseline[i] = value;
    }

    next.checksum = retainedChecksum(next);

    // Publish magic last so an interrupted write is rejected on restore.
    retainedFsrBaseline.magic = 0;
    retainedFsrBaseline.version = next.version;
    retainedFsrBaseline.count = next.count;
    memcpy(
        retainedFsrBaseline.electricalBaseline,
        next.electricalBaseline,
        sizeof(next.electricalBaseline)
    );
    retainedFsrBaseline.checksum = next.checksum;
    retainedFsrBaseline.magic = next.magic;

    return true;
}


bool safeSeatRecoveryLoadFsrBaseline(
    float *outElectricalBaseline,
    size_t count
)
{
    if (
        outElectricalBaseline == nullptr
        || count != SAFESEAT_RETAINED_FSR_COUNT
    )
    {
        return false;
    }

    // Reuse the empty-seat calibration after runtime failures that do not
    // remove power from the hardware. This includes our controlled supervisor
    // restart as well as panic/watchdog resets. Cold boots, brownouts,
    // deep-sleep wakeups and external/manual resets still perform the normal
    // empty-seat calibration so a stale physical setup is never trusted.
    const esp_reset_reason_t resetReason = esp_reset_reason();

    const bool supervisorRestart =
        supervisorRestartMarker == SUPERVISOR_RESTART_MAGIC
        && resetReason == ESP_RST_SW;

    const bool unexpectedRuntimeRestart =
        resetReason == ESP_RST_PANIC
        || resetReason == ESP_RST_INT_WDT
        || resetReason == ESP_RST_TASK_WDT
        || resetReason == ESP_RST_WDT;

    if (
        (!supervisorRestart && !unexpectedRuntimeRestart)
        || !retainedBaselineLooksStructurallyValid()
    )
    {
        return false;
    }

    memcpy(
        outElectricalBaseline,
        retainedFsrBaseline.electricalBaseline,
        sizeof(retainedFsrBaseline.electricalBaseline)
    );

    return true;
}


void safeSeatRecoveryClearRestartRequest()
{
    supervisorRestartMarker = 0;
}


bool safeSeatRecoveryWasSupervisorRestart()
{
    return
        supervisorRestartMarker == SUPERVISOR_RESTART_MAGIC
        && esp_reset_reason() == ESP_RST_SW;
}


bool safeSeatSupervisorStart(unsigned long freezeTimeoutMs)
{
    if (supervisorTaskHandle != nullptr)
    {
        return true;
    }

    if (freezeTimeoutMs < 10000UL)
    {
        freezeTimeoutMs = 10000UL;
    }

    configuredFreezeTimeoutMs = freezeTimeoutMs;
    lastLoopHeartbeatMillis = millis();

    const BaseType_t result = xTaskCreatePinnedToCore(
        supervisorTask,
        "SafeSeatSupervisor",
        2048,
        nullptr,
        2,
        &supervisorTaskHandle,
        0
    );

    if (result != pdPASS)
    {
        supervisorTaskHandle = nullptr;
        configuredFreezeTimeoutMs = 0;
        return false;
    }

    return true;
}


void safeSeatSupervisorHeartbeat()
{
    lastLoopHeartbeatMillis = millis();
}


int safeSeatResetReasonCode()
{
    return static_cast<int>(esp_reset_reason());
}


const char* safeSeatResetReasonText()
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:
            return "POWER_ON";
        case ESP_RST_EXT:
            return "EXTERNAL_RESET";
        case ESP_RST_SW:
            return "SOFTWARE_RESTART";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INTERRUPT_WATCHDOG";
        case ESP_RST_TASK_WDT:
            return "TASK_WATCHDOG";
        case ESP_RST_WDT:
            return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP:
            return "DEEP_SLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "UNKNOWN";
    }
}
