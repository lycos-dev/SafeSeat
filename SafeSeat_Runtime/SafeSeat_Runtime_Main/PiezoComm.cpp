#include "PiezoComm.h"

#include "Config.h"

bool PiezoComm::begin()
{
    latestPacket = PiezoWirePacket{};
    status = PiezoRemoteStatus{};
    packetReceived = false;

    const bool wirelessReady =
        SafeSeatNow::instance().begin();

    status.initialized = wirelessReady;

    return wirelessReady;
}

void PiezoComm::update()
{
    if (!status.initialized)
    {
        return;
    }

    SafeSeatNow &wireless =
        SafeSeatNow::instance();

    wireless.update();

    PiezoWirePacket candidate;
    uint8_t sourceMac[6]{};

    if (
        wireless.takeLatestPiezoPacket(
            candidate,
            sourceMac
        )
    )
    {
        processPacket(
            candidate,
            sourceMac
        );
    }

    const unsigned long now = millis();

    if (packetReceived)
    {
        status.packetAgeMillis =
            now - status.lastPacketMillis;

        status.connected =
            status.packetAgeMillis
            <=
            PIEZO_COMM_FRESHNESS_TIMEOUT_MS;
    }
    else
    {
        status.packetAgeMillis = 0;
        status.connected = false;
    }

    status.linkChannel =
        wireless.getStatus().channel;
}

void PiezoComm::processPacket(
    const PiezoWirePacket &packet,
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
    status.lastSequence =
        latestPacket.sequence;

    status.lastPacketMillis = millis();
    status.packetAgeMillis = 0;
    status.connected = true;

    status.remoteSamplingRateHz =
        latestPacket.actualSamplingRateHz;

    status.remoteFeatureWindowCount =
        latestPacket.featureWindowCount;

    memcpy(
        status.sourceMac,
        sourceMac,
        6
    );
}

bool PiezoComm::packetIsValid(
    const PiezoWirePacket &packet
) const
{
    if (
        packet.magic
        !=
        PIEZO_WIRE_MAGIC
    )
    {
        return false;
    }

    if (
        packet.version
        !=
        PIEZO_WIRE_VERSION
    )
    {
        return false;
    }

    if (
        packet.packetSize
        !=
        sizeof(PiezoWirePacket)
    )
    {
        return false;
    }

    return
        packet.checksum
        ==
        piezoPacketChecksum(packet);
}

PiezoFusionEvidence
PiezoComm::getFusionEvidence() const
{
    PiezoFusionEvidence evidence;

    evidence.available =
        status.initialized;

    evidence.connected =
        status.connected;

    evidence.lastUpdateMillis =
        status.lastPacketMillis;

    if (
        !status.connected
        ||
        !packetReceived
    )
    {
        return evidence;
    }

    const uint16_t flags =
        latestPacket.flags;

    evidence.valid =
        (
            flags
            &
            PIEZO_FLAG_SENSOR_VALID
        )
        !=
        0;

    evidence.signalQualityValid =
        (
            flags
            &
            PIEZO_FLAG_SIGNAL_QUALITY_VALID
        )
        !=
        0;

    if (
        flags
        &
        PIEZO_FLAG_PEAK_RR_VALID
    )
    {
        evidence.peakRespirationBPM =
            latestPacket.peakRespirationBPM;
    }

    if (
        flags
        &
        PIEZO_FLAG_SPECTRAL_RR_VALID
    )
    {
        evidence.spectralRespirationBPM =
            latestPacket.spectralRespirationBPM;
    }

    evidence.noBreathTimerExceeded =
        (
            flags
            &
            PIEZO_FLAG_NO_BREATH_TIMER
        )
        !=
        0;

    evidence.model.available = true;

    evidence.model.valid =
        (
            flags
            &
            PIEZO_FLAG_MODEL_VALID
        )
        !=
        0;

    if (evidence.model.valid)
    {
        evidence.model.isolationForestAnomaly =
            (
                flags
                &
                PIEZO_FLAG_IF_ANOMALY
            )
            !=
            0;

        evidence.model.oneClassSVMAnomaly =
            (
                flags
                &
                PIEZO_FLAG_SVM_ANOMALY
            )
            !=
            0;

        evidence.model.bothModelsAnomaly =
            (
                flags
                &
                PIEZO_FLAG_BOTH_ANOMALY
            )
            !=
            0;

        evidence.model.eitherModelAnomaly =
            (
                flags
                &
                PIEZO_FLAG_EITHER_ANOMALY
            )
            !=
            0;

        evidence.model.isolationForestScore =
            latestPacket.isolationForestDecision;

        evidence.model.oneClassSVMScore =
            latestPacket.oneClassSVMDecision;

        // No score-to-probability conversion is invented.
        evidence.model.confidence = 1.0f;
    }

    return evidence;
}
