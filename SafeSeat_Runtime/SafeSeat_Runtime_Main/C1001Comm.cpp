#include "C1001Comm.h"

#include "Config.h"

bool C1001Comm::begin()
{
    latestPacket = C1001WirePacket{};
    status = C1001RemoteStatus{};
    reading = C1001Reading{};
    model = ModelEvidence{};
    packetReceived = false;

    const bool wirelessReady = SafeSeatNow::instance().begin();
    status.initialized = wirelessReady;

    return wirelessReady;
}

void C1001Comm::update()
{
    if (!status.initialized)
    {
        return;
    }

    SafeSeatNow &wireless = SafeSeatNow::instance();
    wireless.update();

    C1001WirePacket candidate;
    uint8_t sourceMac[6]{};

    if (wireless.takeLatestC1001Packet(candidate, sourceMac))
    {
        processPacket(candidate, sourceMac);
    }

    const unsigned long now = millis();

    if (packetReceived)
    {
        status.packetAgeMillis = now - status.lastPacketMillis;
        status.connected =
            status.packetAgeMillis <= C1001_COMM_FRESHNESS_TIMEOUT_MS;
    }
    else
    {
        status.packetAgeMillis = 0;
        status.connected = false;
    }

    status.linkChannel = wireless.getStatus().channel;

    if (!status.connected)
    {
        invalidateStaleEvidence();
    }
}

void C1001Comm::invalidateStaleEvidence()
{
    // Never let the Main Hub interpret stale remote vitals/model
    // results as current evidence. Preserve only diagnostics such
    // as packet counters/MAC in C1001RemoteStatus.
    reading.connected = false;
    reading.present = false;
    reading.trustedVitalsAvailable = false;
    reading.motionArtifactActive = false;
    reading.status = C1001Status::DISCONNECTED;

    model.valid = false;
    model.isolationForestAnomaly = false;
    model.oneClassSVMAnomaly = false;
    model.bothModelsAnomaly = false;
    model.eitherModelAnomaly = false;
    model.confidence = 0.0f;
}

void C1001Comm::processPacket(
    const C1001WirePacket &packet,
    const uint8_t sourceMac[6]
)
{
    if (!packetIsValid(packet))
    {
        status.badPackets++;
        return;
    }

    latestPacket = packet;
    packetReceived = true;

    status.packetsReceived++;
    status.lastSequence = packet.sequence;
    status.lastPacketMillis = millis();
    status.packetAgeMillis = 0;
    status.connected = true;
    status.remoteMLStatus = packet.mlStatus;
    status.remoteWindowSamplesCollected = packet.windowSamplesCollected;
    status.remoteSamplesUntilNextInference = packet.samplesUntilNextInference;
    status.remoteWindowSamplesRequired = packet.windowSamplesRequired;
    status.remoteWindowsEvaluated = packet.windowsEvaluated;
    memcpy(status.sourceMac, sourceMac, 6);

    const uint16_t flags = packet.flags;

    reading = C1001Reading{};
    reading.connected =
        (flags & C1001_FLAG_SENSOR_CONNECTED) != 0;
    reading.present =
        (flags & C1001_FLAG_PRESENT) != 0;
    reading.warmedUp =
        (flags & C1001_FLAG_WARMED_UP) != 0;
    reading.trustedVitalsAvailable =
        (flags & C1001_FLAG_TRUSTED_VITALS) != 0;
    reading.motionArtifactActive =
        (flags & C1001_FLAG_MOTION_ARTIFACT) != 0;
    reading.validRespiration =
        (flags & C1001_FLAG_VALID_RR) != 0;
    reading.validHeartRate =
        (flags & C1001_FLAG_VALID_HR) != 0;
    reading.validPair =
        (flags & C1001_FLAG_VALID_PAIR) != 0;

    reading.status = static_cast<C1001Status>(packet.sensorStatus);
    reading.motion = packet.motion;
    reading.moveRange = packet.moveRange;
    reading.rawRespiration = packet.rawRespiration;
    reading.rawHeartRate = packet.rawHeartRate;
    reading.medianRespiration = packet.medianRespiration;
    reading.medianHeartRate = packet.medianHeartRate;
    reading.filteredRespiration = packet.filteredRespiration;
    reading.filteredHeartRate = packet.filteredHeartRate;
    reading.warmupRemainingSeconds = packet.warmupRemainingSeconds;
    reading.sampleSequence = packet.sampleSequence;
    reading.sampleTimestampMillis = packet.senderMillis;

    model = ModelEvidence{};
    model.available =
        (flags & C1001_FLAG_MODEL_AVAILABLE) != 0;
    model.valid =
        (flags & C1001_FLAG_MODEL_VALID) != 0;

    if (model.valid)
    {
        model.isolationForestAnomaly =
            (flags & C1001_FLAG_IF_ANOMALY) != 0;
        model.oneClassSVMAnomaly =
            (flags & C1001_FLAG_SVM_ANOMALY) != 0;
        model.bothModelsAnomaly =
            (flags & C1001_FLAG_BOTH_ANOMALY) != 0;
        model.eitherModelAnomaly =
            (flags & C1001_FLAG_EITHER_ANOMALY) != 0;
        model.isolationForestScore = packet.isolationForestDecision;
        model.oneClassSVMScore = packet.oneClassSVMDecision;
        model.confidence = 1.0f;
    }
}

bool C1001Comm::packetIsValid(const C1001WirePacket &packet) const
{
    if (packet.magic != C1001_WIRE_MAGIC) return false;
    if (packet.version != C1001_WIRE_VERSION) return false;
    if (packet.packetSize != sizeof(C1001WirePacket)) return false;
    return packet.checksum == c1001PacketChecksum(packet);
}

C1001FusionInput C1001Comm::getFusionInput() const
{
    C1001FusionInput input;

    if (!status.initialized || !status.connected)
    {
        input.health = FusionSensorHealth::UNAVAILABLE;
        return input;
    }

    input.reading = reading;
    input.model = model;

    if (!reading.connected)
    {
        input.health = FusionSensorHealth::UNAVAILABLE;
    }
    else if (reading.status == C1001Status::WARMING_UP)
    {
        input.health = FusionSensorHealth::WARMING_UP;
    }
    else if (!reading.trustedVitalsAvailable)
    {
        input.health = FusionSensorHealth::DEGRADED;
    }
    else
    {
        input.health = FusionSensorHealth::VALID;
    }

    return input;
}
