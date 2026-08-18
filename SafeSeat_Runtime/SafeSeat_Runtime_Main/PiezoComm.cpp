#include "PiezoComm.h"

#include "Config.h"

bool PiezoComm::begin()
{
    latestPacket = PiezoWirePacket{};
    status = PiezoRemoteStatus{};
    packetReceived = false;

    const bool wirelessReady =
        SafeSeatNow::instance().begin();

    status.initialized =
        wirelessReady;

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

    const unsigned long now =
        millis();

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

    latestPacket =
        packet;

    packetReceived =
        true;

    status.packetsReceived++;

    status.lastSequence =
        latestPacket.sequence;

    status.lastPacketMillis =
        millis();

    status.packetAgeMillis =
        0;

    status.connected =
        true;

    status.remoteSamplingRateHz =
        latestPacket.actualSamplingRateHz;

    status.remoteBreathCount =
        latestPacket.totalBreaths;

    status.remoteSampleCount =
        latestPacket.sampleCount;

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
        piezoPacketChecksum(
            packet
        );
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

    evidence.signalUsable =
        (
            flags
            &
            PIEZO_FLAG_SIGNAL_USABLE
        )
        !=
        0;

    evidence.breathTrackingReady =
        (
            flags
            &
            PIEZO_FLAG_BREATH_TRACKING_READY
        )
        !=
        0;

    evidence.breathDetectedRecently =
        (
            flags
            &
            PIEZO_FLAG_BREATH_DETECTED_RECENT
        )
        !=
        0;

    evidence.noBreathTimerExceeded =
        (
            flags
            &
            PIEZO_FLAG_NO_BREATH_TIMER
        )
        !=
        0;

    if (
        flags
        &
        PIEZO_FLAG_RR_VALID
    )
    {
        evidence.respirationBPM =
            latestPacket.respirationBPM;
    }

    evidence.respirationWave =
        latestPacket.respirationWave;

    evidence.totalBreaths =
        latestPacket.totalBreaths;

    evidence.noBreathDurationMs =
        latestPacket.noBreathDurationMs;

    return evidence;
}
