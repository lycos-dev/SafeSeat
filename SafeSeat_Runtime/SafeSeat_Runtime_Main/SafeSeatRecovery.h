#pragma once

#include <Arduino.h>
#include <stddef.h>

// ============================================================
// SAFESEAT MAIN-HUB FREEZE RECOVERY
//
// - A tiny supervisor task watches for main-loop progress.
// - If loop() makes no progress for the configured timeout, the
//   supervisor performs a controlled ESP restart.
// - The FSR empty-seat electrical baseline is kept in RTC memory.
//   It is restored after a supervisor restart or an unexpected panic/watchdog
//   runtime reset, so an occupied passenger does not need to leave the seat
//   just to recalibrate after a firmware failure.
// - RTC memory is not used as permanent calibration storage. A true power
//   cycle, brownout, external reset, or deep-sleep wake still performs the
//   normal empty-seat calibration.
// ============================================================

static constexpr size_t SAFESEAT_RETAINED_FSR_COUNT = 9;

bool safeSeatRecoverySaveFsrBaseline(
    const float *electricalBaseline,
    size_t count
);

bool safeSeatRecoveryLoadFsrBaseline(
    float *outElectricalBaseline,
    size_t count
);

void safeSeatRecoveryClearRestartRequest();
bool safeSeatRecoveryWasSupervisorRestart();

bool safeSeatSupervisorStart(unsigned long freezeTimeoutMs);
void safeSeatSupervisorHeartbeat();

const char* safeSeatResetReasonText();
int safeSeatResetReasonCode();
