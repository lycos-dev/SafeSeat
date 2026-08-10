#include "C1001Comm.h"

#include <esp_wifi.h>
#include <math.h>

#include "Config.h"

C1001Comm* C1001Comm::activeInstance = nullptr;

const uint8_t C1001Comm::BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

bool C1001Comm::begin()
{
    if (initialized)
    {
        return true;
    }

    WiFi.mode(WIFI_STA);

    const unsigned long waitStart = millis();
    while (!WiFi.STA.started() && millis() - waitStart < 2000UL)
    {
        delay(10);
    }

    if (!WiFi.STA.started())
    {
        return false;
    }

    currentChannel = SAFESEAT_ESPNOW_DEFAULT_CHANNEL;

    if (
        WiFi.setChannel(currentChannel, WIFI_SECOND_CHAN_NONE)
        != ESP_OK
    )
    {
        return false;
    }

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    activeInstance = this;

    if (esp_now_register_recv_cb(&C1001Comm::onReceiveStatic) != ESP_OK)
    {
        return false;
    }

    if (!ensureBroadcastPeer())
    {
        return false;
    }

    initialized = true;
    lastSendMillis = 0;
    packetsSent = 0;
    sendErrors = 0;
    sequence = 0;
    hubLocked = false;
    lastHubBeaconMillis = 0;
    hubBeaconsReceived = 0;
    nextScanChannel = currentChannel;
    lastChannelHopMillis = millis();

    return true;
}

bool C1001Comm::ensureBroadcastPeer()
{
    if (esp_now_is_peer_exist(BROADCAST_MAC))
    {
        return true;
    }

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t result = esp_now_add_peer(&peer);

    return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

unsigned long C1001Comm::getBeaconAgeMillis() const
{
    if (lastHubBeaconMillis == 0)
    {
        return 0;
    }

    return millis() - lastHubBeaconMillis;
}

void C1001Comm::maintainHubChannel()
{
    const unsigned long now = millis();

    if (
        hubLocked
        && lastHubBeaconMillis != 0
        && now - lastHubBeaconMillis <= SAFESEAT_ESPNOW_HUB_BEACON_TIMEOUT_MS
    )
    {
        return;
    }

    hubLocked = false;

    if (now - lastChannelHopMillis >= SAFESEAT_ESPNOW_CHANNEL_DWELL_MS)
    {
        lastChannelHopMillis = now;
        hopToNextChannel();
    }
}

void C1001Comm::hopToNextChannel()
{
    if (
        nextScanChannel < SAFESEAT_ESPNOW_MIN_CHANNEL
        || nextScanChannel > SAFESEAT_ESPNOW_MAX_CHANNEL
    )
    {
        nextScanChannel = SAFESEAT_ESPNOW_MIN_CHANNEL;
    }
    else
    {
        nextScanChannel++;
        if (nextScanChannel > SAFESEAT_ESPNOW_MAX_CHANNEL)
        {
            nextScanChannel = SAFESEAT_ESPNOW_MIN_CHANNEL;
        }
    }

    if (
        WiFi.setChannel(nextScanChannel, WIFI_SECOND_CHAN_NONE)
        == ESP_OK
    )
    {
        currentChannel = nextScanChannel;
    }
}

void C1001Comm::onReceiveStatic(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
)
{
    if (activeInstance != nullptr)
    {
        activeInstance->onReceive(info, data, len);
    }
}

void C1001Comm::onReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
)
{
    (void)info;

    if (
        data == nullptr
        || len != static_cast<int>(sizeof(SafeSeatHubBeacon))
    )
    {
        return;
    }

    SafeSeatHubBeacon beacon;
    memcpy(&beacon, data, sizeof(beacon));

    if (
        beacon.magic != SAFESEAT_HUB_BEACON_MAGIC
        || beacon.version != SAFESEAT_NOW_PROTOCOL_VERSION
        || beacon.packetSize != sizeof(SafeSeatHubBeacon)
        || beacon.checksum != safeSeatHubBeaconChecksum(beacon)
        || beacon.channel < SAFESEAT_ESPNOW_MIN_CHANNEL
        || beacon.channel > SAFESEAT_ESPNOW_MAX_CHANNEL
    )
    {
        return;
    }

    currentChannel = beacon.channel;
    nextScanChannel = beacon.channel;
    lastHubBeaconMillis = millis();
    hubBeaconsReceived++;
    hubLocked = true;
}

void C1001Comm::update(
    const C1001Reading &sensorReading,
    const C1001MLReading &modelReading
)
{
    if (!initialized)
    {
        return;
    }

    maintainHubChannel();

    const unsigned long now = millis();
    if (now - lastSendMillis < C1001_COMM_TX_INTERVAL_MS)
    {
        return;
    }

    lastSendMillis = now;

    C1001WirePacket packet;
    packet.sequence = ++sequence;
    packet.senderMillis = now;
    packet.flags = 0;

    if (sensorReading.connected) packet.flags |= C1001_FLAG_SENSOR_CONNECTED;
    if (sensorReading.present) packet.flags |= C1001_FLAG_PRESENT;
    if (sensorReading.warmedUp) packet.flags |= C1001_FLAG_WARMED_UP;
    if (sensorReading.trustedVitalsAvailable) packet.flags |= C1001_FLAG_TRUSTED_VITALS;
    if (sensorReading.motionArtifactActive) packet.flags |= C1001_FLAG_MOTION_ARTIFACT;
    if (sensorReading.validRespiration) packet.flags |= C1001_FLAG_VALID_RR;
    if (sensorReading.validHeartRate) packet.flags |= C1001_FLAG_VALID_HR;
    if (sensorReading.validPair) packet.flags |= C1001_FLAG_VALID_PAIR;

    if (modelReading.modelAvailable) packet.flags |= C1001_FLAG_MODEL_AVAILABLE;
    if (modelReading.valid) packet.flags |= C1001_FLAG_MODEL_VALID;
    if (modelReading.isolationForestAnomaly) packet.flags |= C1001_FLAG_IF_ANOMALY;
    if (modelReading.oneClassSVMAnomaly) packet.flags |= C1001_FLAG_SVM_ANOMALY;
    if (modelReading.bothModelsAnomaly) packet.flags |= C1001_FLAG_BOTH_ANOMALY;
    if (modelReading.eitherModelAnomaly) packet.flags |= C1001_FLAG_EITHER_ANOMALY;

    packet.sensorStatus = static_cast<uint8_t>(sensorReading.status);
    packet.mlStatus = static_cast<uint8_t>(modelReading.status);
    packet.motion = static_cast<int16_t>(sensorReading.motion);
    packet.moveRange = static_cast<int16_t>(sensorReading.moveRange);
    packet.rawRespiration = static_cast<int16_t>(sensorReading.rawRespiration);
    packet.rawHeartRate = static_cast<int16_t>(sensorReading.rawHeartRate);
    packet.medianRespiration = static_cast<int16_t>(sensorReading.medianRespiration);
    packet.medianHeartRate = static_cast<int16_t>(sensorReading.medianHeartRate);
    packet.filteredRespiration = sensorReading.filteredRespiration;
    packet.filteredHeartRate = sensorReading.filteredHeartRate;
    packet.warmupRemainingSeconds = static_cast<uint16_t>(sensorReading.warmupRemainingSeconds);
    packet.windowSamplesCollected = modelReading.windowSamplesCollected;
    packet.samplesUntilNextInference = modelReading.samplesUntilNextInference;
    packet.windowSamplesRequired = modelReading.windowSamplesRequired;
    packet.sampleSequence = sensorReading.sampleSequence;
    packet.windowsEvaluated = modelReading.windowsEvaluated;
    packet.isolationForestDecision = modelReading.isolationForestDecision;
    packet.oneClassSVMDecision = modelReading.oneClassSVMDecision;

    packet.checksum = 0;
    packet.checksum = c1001PacketChecksum(packet);

    const esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&packet),
        sizeof(packet)
    );

    if (result == ESP_OK)
    {
        packetsSent++;
    }
    else
    {
        sendErrors++;
    }
}
