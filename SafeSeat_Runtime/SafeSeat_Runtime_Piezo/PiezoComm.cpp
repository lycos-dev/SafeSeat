#include "PiezoComm.h"

#include "Config.h"

bool PiezoComm::begin()
{
    // Send-only UART. RX is intentionally disabled.
    Serial2.begin(
        PIEZO_COMM_BAUD,
        SERIAL_8N1,
        -1,
        PIEZO_COMM_TX_PIN
    );

    initialized = true;
    lastSendMillis = 0;
    packetsSent = 0;
    sequence = 0;

    return true;
}


void PiezoComm::update(
    const PiezoReading &reading,
    bool signalWindowAligned,
    const PiezoSignalQuality &signalQuality,
    bool featureVectorReady,
    bool inferenceReady,
    const PiezoFeatures &features,
    const PiezoInferenceResult &inference,
    unsigned long featureWindowCount
)
{
    if (!initialized)
    {
        return;
    }

    const unsigned long now = millis();

    if (
        now - lastSendMillis
        <
        PIEZO_COMM_TX_INTERVAL_MS
    )
    {
        return;
    }

    lastSendMillis = now;

    PiezoWirePacket packet;
    packet.sequence = ++sequence;
    packet.senderMillis = now;
    packet.flags = 0;

    if (reading.valid)
    {
        packet.flags |= PIEZO_FLAG_SENSOR_VALID;
    }

    if (
        signalWindowAligned
        &&
        signalQuality.valid
    )
    {
        packet.flags |= PIEZO_FLAG_SIGNAL_QUALITY_VALID;
    }

    if (featureVectorReady)
    {
        packet.flags |= PIEZO_FLAG_FEATURE_READY;
    }

    if (
        isfinite(
            reading.estimatedRespirationBPM
        )
    )
    {
        packet.flags |= PIEZO_FLAG_PEAK_RR_VALID;
        packet.peakRespirationBPM =
            reading.estimatedRespirationBPM;
    }
    else
    {
        packet.peakRespirationBPM = NAN;
    }

    if (
        featureVectorReady
        &&
        isfinite(
            features.respirationBPM
        )
    )
    {
        packet.flags |= PIEZO_FLAG_SPECTRAL_RR_VALID;
        packet.spectralRespirationBPM =
            features.respirationBPM;
    }
    else
    {
        packet.spectralRespirationBPM = NAN;
    }

    if (reading.noBreathTimerExceeded)
    {
        // Auxiliary engineering timer only.
        // The Main Hub does not treat this flag as a standalone
        // medical-emergency vote.
        packet.flags |= PIEZO_FLAG_NO_BREATH_TIMER;
    }

    if (
        inferenceReady
        &&
        inference.valid
        &&
        signalWindowAligned
        &&
        signalQuality.valid
    )
    {
        packet.flags |= PIEZO_FLAG_MODEL_VALID;

        if (inference.isolationForestAnomaly)
        {
            packet.flags |= PIEZO_FLAG_IF_ANOMALY;
        }

        if (inference.oneClassSVMAnomaly)
        {
            packet.flags |= PIEZO_FLAG_SVM_ANOMALY;
        }

        if (inference.bothModelsAnomaly)
        {
            packet.flags |= PIEZO_FLAG_BOTH_ANOMALY;
        }

        if (inference.eitherModelAnomaly)
        {
            packet.flags |= PIEZO_FLAG_EITHER_ANOMALY;
        }

        packet.isolationForestDecision =
            inference.isolationForestDecision;

        packet.oneClassSVMDecision =
            inference.oneClassSVMDecision;
    }
    else
    {
        packet.isolationForestDecision = NAN;
        packet.oneClassSVMDecision = NAN;
    }

    packet.actualSamplingRateHz =
        reading.actualSamplingRateHz;

    packet.featureWindowCount =
        static_cast<uint32_t>(
            featureWindowCount
        );

    packet.checksum = 0;
    packet.checksum =
        piezoPacketChecksum(
            packet
        );

    const uint8_t *bytes =
        reinterpret_cast<const uint8_t *>(
            &packet
        );

    const size_t written =
        Serial2.write(
            bytes,
            sizeof(packet)
        );

    if (
        written
        ==
        sizeof(packet)
    )
    {
        packetsSent++;
    }
}
