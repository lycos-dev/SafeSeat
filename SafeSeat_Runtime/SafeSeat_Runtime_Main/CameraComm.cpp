#include "CameraComm.h"

#include <esp_system.h>
#include "Config.h"

bool CameraComm::begin()
{
    transport = &SafeSeatNow::instance();
    if (!transport->begin()) return false;

    status = CameraRemoteStatus{};
    status.initialized = true;
    occupancyCandidateSince = millis();
    return true;
}

uint32_t CameraComm::allocateRequestId()
{
    if (nextRequestId == 0) nextRequestId = 1;
    return nextRequestId++;
}

void CameraComm::update()
{
    if (!status.initialized || transport == nullptr) return;

    CameraStatusPacket statusPacket;
    uint8_t sourceMac[6]{};
    while (transport->takeLatestCameraStatus(statusPacket, sourceMac))
    {
        processStatusPacket(statusPacket, sourceMac);
    }

    CameraResultPacket resultPacket;
    while (transport->takeLatestCameraResult(resultPacket, sourceMac))
    {
        processResultPacket(resultPacket, sourceMac);
    }

    const unsigned long now = millis();
    status.packetAgeMillis = lastPacketMillis > 0 ? now - lastPacketMillis : 0UL;
    status.stale = lastPacketMillis > 0 && status.packetAgeMillis > CAMERA_COMM_STALE_AFTER_MS;
    status.connected = lastPacketMillis > 0 && status.packetAgeMillis <= CAMERA_COMM_DISCONNECT_TIMEOUT_MS;

    status.localSessionId = sessionId;
    status.localOccupancySessionActive = occupancyStable && sessionId != 0;

    if (status.requestActive)
    {
        status.requestAgeMillis = now - requestStartMillis;
        if (status.requestAgeMillis > CAMERA_REQUEST_TIMEOUT_MS)
        {
            clearActiveRequest();
            lastRequestFinishedMillis = now;
        }
    }
    else
    {
        status.requestAgeMillis = 0UL;
    }

    if (hasAcceptedResult && now - acceptedResultMillis > CAMERA_RESULT_FRESHNESS_TIMEOUT_MS)
    {
        hasAcceptedResult = false;
    }

    serviceSessionCommands();
}

void CameraComm::processStatusPacket(const CameraStatusPacket &packet, const uint8_t sourceMac[6])
{
    if (packet.magic != CAMERA_STATUS_MAGIC
        || packet.version != CAMERA_WIRE_VERSION
        || packet.packetSize != sizeof(CameraStatusPacket)
        || packet.checksum != cameraStatusChecksum(packet))
    {
        status.badPackets++;
        return;
    }

    lastPacketMillis = millis();
    memcpy(status.sourceMac, sourceMac, 6);
    status.channel = packet.channel;
    status.remoteSessionId = packet.sessionId;
    status.lastHandledRequestId = packet.lastHandledRequestId;
    status.lastRemoteInferenceMillis = packet.lastInferenceMillis;
    status.calibrationCount = packet.calibrationCount;
    status.calibrationTarget = packet.calibrationTarget;
    status.modelReady = (packet.flags & CAMERA_STATUS_MODEL_READY) != 0;
    status.cameraReady = (packet.flags & CAMERA_STATUS_CAMERA_READY) != 0;
    status.psramReady = (packet.flags & CAMERA_STATUS_PSRAM_READY) != 0;
    status.busy = (packet.flags & CAMERA_STATUS_BUSY) != 0;
    status.baselineReady = (packet.flags & CAMERA_STATUS_BASELINE_READY) != 0;
    status.baselineProvisional = (packet.flags & CAMERA_STATUS_BASELINE_PROVISIONAL) != 0;
    status.calibrating = (packet.flags & CAMERA_STATUS_CALIBRATING) != 0;
    status.sessionActive = (packet.flags & CAMERA_STATUS_SESSION_ACTIVE) != 0;
    status.statusPacketsReceived++;

    if (calibrationCommandPending
        && packet.lastHandledRequestId == calibrationRequestId
        && packet.sessionId == sessionId
        && (status.calibrating || status.baselineReady))
    {
        calibrationCommandPending = false;
    }

    if (resetCommandPending
        && packet.lastHandledRequestId == resetRequestId
        && !status.sessionActive
        && packet.sessionId == 0)
    {
        resetCommandPending = false;
        closingSessionId = 0;
    }
}

void CameraComm::processResultPacket(const CameraResultPacket &packet, const uint8_t sourceMac[6])
{
    if (packet.magic != CAMERA_RESULT_MAGIC
        || packet.version != CAMERA_WIRE_VERSION
        || packet.packetSize != sizeof(CameraResultPacket)
        || packet.checksum != cameraResultChecksum(packet))
    {
        status.badPackets++;
        return;
    }

    lastPacketMillis = millis();
    memcpy(status.sourceMac, sourceMac, 6);
    status.resultPacketsReceived++;

    if (!status.requestActive
        || packet.requestId != status.activeRequestId
        || packet.sessionId != sessionId
        || packet.sequence == 0)
    {
        status.ignoredResults++;
        return;
    }

    const CameraPostureClass decision = static_cast<CameraPostureClass>(packet.postureClass);
    status.lastResultRequestId = packet.requestId;
    status.lastPosture = decision;
    status.lastConfidence = packet.confidenceMilli / 1000.0f;
    status.lastValidFrames = packet.validFrames;
    status.lastIFScore = packet.ifScore;
    status.lastOCSVMScore = packet.ocsvmScore;
    status.lastRemoteInferenceMillis = packet.inferenceMillis;

    // PENDING is intentionally not a Fusion vote. Keep the request open
    // so the camera can perform its second valid abnormal observation.
    if (decision == CameraPostureClass::DEVIATION_PENDING) return;

    // Final valid results are the only packets exposed to Fusion.
    if ((packet.flags & CAMERA_RESULT_VALID) != 0 && cameraPostureIsFinal(decision))
    {
        acceptedResult = packet;
        acceptedResultMillis = millis();
        hasAcceptedResult = true;
    }

    clearActiveRequest();
    lastRequestFinishedMillis = millis();
}

bool CameraComm::sendCommand(CameraCommandType command, uint32_t requestId, uint32_t commandSessionId)
{
    if (transport == nullptr || requestId == 0) return false;
    CameraCommandPacket packet;
    packet.requestId = requestId;
    packet.sessionId = commandSessionId;
    packet.command = static_cast<uint8_t>(command);
    packet.checksum = 0;
    packet.checksum = cameraCommandChecksum(packet);
    const bool ok = transport->sendCameraCommand(packet);
    if (ok) status.commandPacketsQueued++;
    return ok;
}

void CameraComm::startOccupancySession()
{
    uint32_t candidate = esp_random();
    if (candidate == 0) candidate = 1;
    sessionId = candidate;
    status.localSessionId = sessionId;
    hasAcceptedResult = false;
    clearActiveRequest();

    calibrationRequestId = allocateRequestId();
    calibrationCommandPending = true;
    resetCommandPending = false;
    lastCommandSendMillis = 0;
}

void CameraComm::endOccupancySession()
{
    hasAcceptedResult = false;
    clearActiveRequest();

    closingSessionId = sessionId != 0 ? sessionId : status.remoteSessionId;
    if (closingSessionId != 0)
    {
        resetRequestId = allocateRequestId();
        resetCommandPending = true;
    }

    sessionId = 0;
    status.localSessionId = 0;
    calibrationCommandPending = false;
    lastCommandSendMillis = 0;
}

void CameraComm::serviceOccupancySession(bool occupied)
{
    if (!status.initialized) return;
    const unsigned long now = millis();

    if (occupied != occupancyCandidate)
    {
        occupancyCandidate = occupied;
        occupancyCandidateSince = now;
    }

    const unsigned long required = occupied
        ? CAMERA_OCCUPANCY_ENTER_DEBOUNCE_MS
        : CAMERA_OCCUPANCY_EXIT_DEBOUNCE_MS;

    if (occupancyCandidate != occupancyStable
        && now - occupancyCandidateSince >= required)
    {
        occupancyStable = occupancyCandidate;
        if (occupancyStable) startOccupancySession();
        else endOccupancySession();
    }

    // If the hub boots while the seat is empty and discovers an old camera
    // session, clear it rather than allowing a previous passenger baseline.
    if (!occupancyStable
        && !resetCommandPending
        && status.sessionActive
        && status.remoteSessionId != 0)
    {
        closingSessionId = status.remoteSessionId;
        resetRequestId = allocateRequestId();
        resetCommandPending = true;
        lastCommandSendMillis = 0;
    }
}

void CameraComm::serviceSessionCommands()
{
    if (transport == nullptr) return;
    const unsigned long now = millis();
    if (lastCommandSendMillis != 0 && now - lastCommandSendMillis < CAMERA_SESSION_COMMAND_RETRY_MS) return;

    if (resetCommandPending && closingSessionId != 0)
    {
        sendCommand(CameraCommandType::RESET_SESSION, resetRequestId, closingSessionId);
        lastCommandSendMillis = now;
        return;
    }

    if (occupancyStable && sessionId != 0)
    {
        // If the camera rebooted, lost its session, or has not acknowledged
        // calibration yet, resend the same idempotent CALIBRATE command.
        const bool remoteNeedsSession =
            status.remoteSessionId != sessionId
            || (!status.baselineReady && !status.calibrating);

        if (remoteNeedsSession && !calibrationCommandPending)
        {
            calibrationRequestId = allocateRequestId();
            calibrationCommandPending = true;
        }

        if (calibrationCommandPending)
        {
            sendCommand(CameraCommandType::CALIBRATE_UPRIGHT, calibrationRequestId, sessionId);
            lastCommandSendMillis = now;
        }
    }
}

void CameraComm::startVerificationRequest()
{
    status.activeRequestId = allocateRequestId();
    status.requestActive = true;
    requestStartMillis = millis();
    lastCommandSendMillis = 0;
    hasAcceptedResult = false;
}

void CameraComm::sendActiveVerificationRequest()
{
    if (!status.requestActive || sessionId == 0) return;
    if (sendCommand(CameraCommandType::VERIFY_POSTURE, status.activeRequestId, sessionId))
    {
        lastCommandSendMillis = millis();
    }
}

void CameraComm::clearActiveRequest()
{
    status.requestActive = false;
    status.activeRequestId = 0;
    requestStartMillis = 0;
}

void CameraComm::serviceVerificationRequest(bool requested)
{
    if (!status.initialized || transport == nullptr) return;
    const unsigned long now = millis();

    if (!requested)
    {
        // Late camera results are transaction-checked and safely ignored.
        if (status.requestActive)
        {
            clearActiveRequest();
            lastRequestFinishedMillis = now;
        }
        return;
    }

    const bool currentSessionReady =
        occupancyStable
        && sessionId != 0
        && status.connected
        && !status.stale
        && status.remoteSessionId == sessionId
        && status.sessionActive
        && status.baselineReady
        && status.modelReady
        && status.cameraReady
        && status.psramReady;

    if (!currentSessionReady) return;

    if (!status.requestActive)
    {
        if (lastRequestFinishedMillis != 0
            && now - lastRequestFinishedMillis < CAMERA_REQUEST_COOLDOWN_MS) return;
        startVerificationRequest();
    }

    if (lastCommandSendMillis == 0
        || now - lastCommandSendMillis >= CAMERA_TRIGGER_RETRY_MS)
    {
        sendActiveVerificationRequest();
    }
}

CameraFusionEvidence CameraComm::getFusionEvidence() const
{
    CameraFusionEvidence evidence;
    evidence.available = status.initialized;
    evidence.connected = status.connected
        && !status.stale
        && status.modelReady
        && status.cameraReady
        && status.psramReady;

    if (!hasAcceptedResult) return evidence;

    const CameraPostureClass decision = static_cast<CameraPostureClass>(acceptedResult.postureClass);
    evidence.resultValid =
        evidence.connected
        && (acceptedResult.flags & CAMERA_RESULT_VALID) != 0
        && acceptedResult.sessionId == sessionId
        && cameraPostureIsFinal(decision);

    if (evidence.resultValid)
    {
        evidence.postureNormal = cameraPostureIsNormal(decision);
        evidence.postureAbnormal = cameraPostureIsAbnormal(decision);
        evidence.confidence = acceptedResult.confidenceMilli / 1000.0f;
        evidence.requestId = acceptedResult.requestId;
        evidence.resultId = acceptedResult.sequence;
        evidence.postureClass = acceptedResult.postureClass;
        evidence.lastUpdateMillis = acceptedResultMillis;
    }

    return evidence;
}
