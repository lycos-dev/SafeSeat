#include "PiezoComm.h"

#include "Config.h"

bool PiezoComm::begin()
{
    // Receive-only UART. TX is intentionally disabled.
    Serial2.begin(
        PIEZO_COMM_BAUD,
        SERIAL_8N1,
        PIEZO_COMM_RX_PIN,
        -1
    );

    receiveIndex = 0;
    latestPacket = PiezoWirePacket{};
    status = PiezoRemoteStatus{};
    status.initialized = true;
    packetReceived = false;

    return true;
}


void PiezoComm::update()
{
    if (!status.initialized)
    {
        return;
    }

    while (Serial2.available() > 0)
    {
        int value = Serial2.read();

        if (value < 0)
        {
            break;
        }

        consumeByte(
            static_cast<uint8_t>(
                value
            )
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
}


void PiezoComm::consumeByte(
    uint8_t value
)
{
    const uint8_t magicLow =
        static_cast<uint8_t>(
            PIEZO_WIRE_MAGIC
            &
            0xFFu
        );

    const uint8_t magicHigh =
        static_cast<uint8_t>(
            (
                PIEZO_WIRE_MAGIC
                >>
                8
            )
            &
            0xFFu
        );

    if (receiveIndex == 0)
    {
        if (value == magicLow)
        {
            receiveBuffer[0] = value;
            receiveIndex = 1;
        }

        return;
    }

    if (receiveIndex == 1)
    {
        if (value == magicHigh)
        {
            receiveBuffer[1] = value;
            receiveIndex = 2;
        }
        else if (value == magicLow)
        {
            // Possible new packet start.
            receiveBuffer[0] = value;
            receiveIndex = 1;
        }
        else
        {
            receiveIndex = 0;
        }

        return;
    }

    receiveBuffer[
        receiveIndex++
    ] = value;

    if (
        receiveIndex
        >=
        sizeof(PiezoWirePacket)
    )
    {
        processBufferedPacket();
        receiveIndex = 0;
    }
}


void PiezoComm::processBufferedPacket()
{
    PiezoWirePacket candidate;

    memcpy(
        &candidate,
        receiveBuffer,
        sizeof(candidate)
    );

    if (!packetIsValid(candidate))
    {
        status.badPackets++;
        return;
    }

    latestPacket = candidate;
    packetReceived = true;

    status.packetsReceived++;
    status.lastSequence =
        latestPacket.sequence;

    status.lastPacketMillis =
        millis();

    status.packetAgeMillis = 0;
    status.connected = true;

    status.remoteSamplingRateHz =
        latestPacket.actualSamplingRateHz;

    status.remoteFeatureWindowCount =
        latestPacket.featureWindowCount;
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

    evidence.model.available =
        true;

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
