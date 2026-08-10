#include "CameraComm.h"

#include "Config.h"

bool CameraComm::begin()
{
    transport = &SafeSeatNow::instance();

    if (!transport->begin())
    {
        return false;
    }

    status = CameraRemoteStatus{};
    status.initialized = true;
    return true;
}

void CameraComm::update()
{
    if (!status.initialized || transport == nullptr)
    {
        return;
    }

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

    status.packetAgeMillis =
        lastPacketMillis > 0
            ? now - lastPacketMillis
            : 0UL;

    status.connected =
        lastPacketMillis > 0
        && status.packetAgeMillis <= CAMERA_COMM_FRESHNESS_TIMEOUT_MS;

    if (status.requestActive)
    {
        status.requestAgeMillis = now - requestStartMillis;

        if (status.requestAgeMillis > CAMERA_REQUEST_TIMEOUT_MS)
        {
            clearActiveRequest();
        }
    }
    else
    {
        status.requestAgeMillis = 0UL;
    }

    if (hasAcceptedResult
        && now - acceptedResultMillis > CAMERA_RESULT_FRESHNESS_TIMEOUT_MS)
    {
        hasAcceptedResult = false;
    }
}

void CameraComm::processStatusPacket(
    const CameraStatusPacket &packet,
    const uint8_t sourceMac[6]
)
{
    if (packet.magic != CAMERA_STATUS_MAGIC
        || packet.version != CAMERA_WIRE_VERSION
        || packet.packetSize != sizeof(CameraStatusPacket)
        || packet.checksum != cameraStatusChecksum(packet))
    {
        status.badPackets++;
        return;
    }

    const unsigned long now = millis();
    lastPacketMillis = now;

    memcpy(status.sourceMac, sourceMac, 6);
    status.channel = packet.channel;
    status.modelReady = (packet.flags & CAMERA_STATUS_MODEL_READY) != 0;
    status.cameraReady = (packet.flags & CAMERA_STATUS_CAMERA_READY) != 0;
    status.psramReady = (packet.flags & CAMERA_STATUS_PSRAM_READY) != 0;
    status.busy = (packet.flags & CAMERA_STATUS_BUSY) != 0;
    status.statusPacketsReceived++;
}

void CameraComm::processResultPacket(
    const CameraResultPacket &packet,
    const uint8_t sourceMac[6]
)
{
    if (packet.magic != CAMERA_RESULT_MAGIC
        || packet.version != CAMERA_WIRE_VERSION
        || packet.packetSize != sizeof(CameraResultPacket)
        || packet.checksum != cameraResultChecksum(packet))
    {
        status.badPackets++;
        return;
    }

    const unsigned long now = millis();
    lastPacketMillis = now;
    memcpy(status.sourceMac, sourceMac, 6);
    status.resultPacketsReceived++;

    // Transaction safety: a result is accepted only for the
    // currently active request. Late/stale results cannot clear
    // or confirm a future emergency candidate.
    if (!status.requestActive
        || packet.requestId != status.activeRequestId
        || packet.sequence == 0)
    {
        status.ignoredResults++;
        return;
    }

    lastResultSequence = packet.sequence;
    acceptedResult = packet;
    acceptedResultMillis = now;
    hasAcceptedResult = true;

    status.lastResultRequestId = packet.requestId;
    status.lastPosture = static_cast<CameraPostureClass>(packet.postureClass);
    status.lastConfidence = packet.confidenceMilli / 1000.0f;
    status.lastValidFrames = packet.validFrames;
    status.lastRemoteInferenceMillis = packet.inferenceMillis;

    clearActiveRequest();
}

void CameraComm::startRequest()
{
    if (nextRequestId == 0)
    {
        nextRequestId = 1;
    }

    status.activeRequestId = nextRequestId++;
    status.requestActive = true;
    requestStartMillis = millis();
    lastTriggerSendMillis = 0;

    // A new transaction invalidates any previous result snapshot.
    hasAcceptedResult = false;
}

void CameraComm::sendActiveRequest()
{
    if (!status.requestActive || transport == nullptr)
    {
        return;
    }

    CameraTriggerPacket trigger;
    trigger.requestId = status.activeRequestId;
    trigger.frameCount = CAMERA_VERIFY_FRAME_COUNT;
    trigger.minValidFrames = CAMERA_VERIFY_MIN_VALID_FRAMES;
    trigger.checksum = 0;
    trigger.checksum = cameraTriggerChecksum(trigger);

    if (transport->sendCameraTrigger(trigger))
    {
        status.triggerPacketsQueued++;
    }

    lastTriggerSendMillis = millis();
}

void CameraComm::clearActiveRequest()
{
    status.requestActive = false;
    status.activeRequestId = 0;
    requestStartMillis = 0;
    lastTriggerSendMillis = 0;
}

void CameraComm::serviceVerificationRequest(bool requested)
{
    if (!status.initialized || transport == nullptr)
    {
        return;
    }

    const unsigned long now = millis();

    if (!requested)
    {
        // If Fusion no longer wants a camera result, abandon any
        // outstanding transaction. A late result will be ignored.
        if (status.requestActive)
        {
            clearActiveRequest();
        }
        return;
    }

    if (!status.requestActive)
    {
        startRequest();
    }

    if (lastTriggerSendMillis == 0
        || now - lastTriggerSendMillis >= CAMERA_TRIGGER_RETRY_MS)
    {
        sendActiveRequest();
    }
}

CameraFusionEvidence CameraComm::getFusionEvidence() const
{
    CameraFusionEvidence evidence;
    evidence.available = status.initialized;
    evidence.connected = status.connected
        && status.modelReady
        && status.cameraReady
        && status.psramReady;

    if (!hasAcceptedResult)
    {
        return evidence;
    }

    const bool validFlag =
        (acceptedResult.flags & CAMERA_RESULT_VALID) != 0;

    const CameraPostureClass posture =
        static_cast<CameraPostureClass>(acceptedResult.postureClass);

    evidence.resultValid =
        evidence.connected
        && validFlag
        && acceptedResult.validFrames >= CAMERA_VERIFY_MIN_VALID_FRAMES
        && (cameraPostureIsNormal(posture) || cameraPostureIsAbnormal(posture));

    if (evidence.resultValid)
    {
        evidence.postureNormal = cameraPostureIsNormal(posture);
        evidence.postureAbnormal = cameraPostureIsAbnormal(posture);
        evidence.confidence = acceptedResult.confidenceMilli / 1000.0f;
        evidence.requestId = acceptedResult.requestId;
        evidence.resultId = acceptedResult.sequence;
        evidence.postureClass = acceptedResult.postureClass;
        evidence.lastUpdateMillis = acceptedResultMillis;
    }

    return evidence;
}
