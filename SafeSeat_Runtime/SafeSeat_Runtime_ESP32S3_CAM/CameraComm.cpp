#include "CameraComm.h"

#include <esp_wifi.h>

#include "Config.h"

CameraComm *CameraComm::activeInstance = nullptr;

const uint8_t CameraComm::BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

bool CameraComm::begin()
{
    WiFi.mode(WIFI_STA);
    delay(50);

    setChannel(SAFESEAT_ESPNOW_DEFAULT_CHANNEL);

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    activeInstance = this;

    if (esp_now_register_recv_cb(&CameraComm::onReceiveStatic) != ESP_OK)
    {
        return false;
    }

    if (!ensureBroadcastPeer())
    {
        return false;
    }

    status = CameraNodeLinkStatus{};
    status.initialized = true;
    status.channel = SAFESEAT_ESPNOW_DEFAULT_CHANNEL;
    nextScanChannel = SAFESEAT_ESPNOW_DEFAULT_CHANNEL;
    lastScanStepMillis = millis();
    lastStatusMillis = 0;
    pendingTriggerReady = false;
    lastAcceptedTriggerId = 0;
    stationChannelManaged = false;
    return true;
}

bool CameraComm::ensureBroadcastPeer()
{
    if (esp_now_is_peer_exist(BROADCAST_MAC))
    {
        return true;
    }

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;               // current Wi-Fi channel
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t result = esp_now_add_peer(&peer);
    return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

void CameraComm::setChannel(uint8_t channel)
{
    if (channel < SAFESEAT_ESPNOW_MIN_CHANNEL
        || channel > SAFESEAT_ESPNOW_MAX_CHANNEL)
    {
        return;
    }

    // Never force a channel while STA association/connection owns it.
    if (stationChannelManaged)
    {
        return;
    }

    if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) == ESP_OK)
    {
        status.channel = channel;
    }
}

void CameraComm::serviceChannelDiscovery()
{
    const unsigned long now = millis();

    // When the S3 is associating with or connected to the SafeSeat AP,
    // Wi-Fi owns the radio channel.  ESP-NOW automatically operates on
    // that same channel, so manual scanning must stop.
    if (stationChannelManaged)
    {
        if (WiFi.status() == WL_CONNECTED && WiFi.channel() > 0)
        {
            status.channel = WiFi.channel();
        }
        return;
    }

    if (status.hubLocked
        && now - status.lastHubBeaconMillis <= SAFESEAT_ESPNOW_BEACON_STALE_MS)
    {
        return;
    }

    status.hubLocked = false;

    if (now - lastScanStepMillis < SAFESEAT_ESPNOW_SCAN_DWELL_MS)
    {
        return;
    }

    lastScanStepMillis = now;

    if (nextScanChannel < SAFESEAT_ESPNOW_MIN_CHANNEL
        || nextScanChannel > SAFESEAT_ESPNOW_MAX_CHANNEL)
    {
        nextScanChannel = SAFESEAT_ESPNOW_MIN_CHANNEL;
    }

    setChannel(nextScanChannel);

    nextScanChannel++;
    if (nextScanChannel > SAFESEAT_ESPNOW_MAX_CHANNEL)
    {
        nextScanChannel = SAFESEAT_ESPNOW_MIN_CHANNEL;
    }
}

void CameraComm::update(
    bool modelReady,
    bool cameraReady,
    bool psramReady,
    bool busy,
    bool wifiConnected,
    bool stationOwnsChannel,
    unsigned long lastInferenceMillis,
    uint32_t lastHandledRequestId
)
{
    if (!status.initialized)
    {
        return;
    }

    stationChannelManaged = stationOwnsChannel;
    status.stationOwnsChannel = stationOwnsChannel;
    status.wifiConnected = wifiConnected;

    serviceChannelDiscovery();

    const unsigned long now = millis();
    if (now - lastStatusMillis >= CAMERA_STATUS_INTERVAL_MS)
    {
        lastStatusMillis = now;
        sendStatus(
            modelReady,
            cameraReady,
            psramReady,
            busy,
            lastInferenceMillis,
            lastHandledRequestId
        );
    }
}

void CameraComm::sendStatus(
    bool modelReady,
    bool cameraReady,
    bool psramReady,
    bool busy,
    unsigned long lastInferenceMillis,
    uint32_t lastHandledRequestId
)
{
    CameraStatusPacket packet;
    packet.sequence = ++statusSequence;
    packet.channel = status.channel;
    packet.lastInferenceMillis = lastInferenceMillis;
    packet.freeHeapBytes = ESP.getFreeHeap();
    packet.lastHandledRequestId = lastHandledRequestId;

    if (modelReady) packet.flags |= CAMERA_STATUS_MODEL_READY;
    if (cameraReady) packet.flags |= CAMERA_STATUS_CAMERA_READY;
    if (psramReady) packet.flags |= CAMERA_STATUS_PSRAM_READY;
    if (busy) packet.flags |= CAMERA_STATUS_BUSY;

    packet.checksum = 0;
    packet.checksum = cameraStatusChecksum(packet);

    const esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&packet),
        sizeof(packet)
    );

    if (result == ESP_OK)
    {
        status.statusPacketsSent++;
    }
    else
    {
        status.sendErrors++;
    }
}

bool CameraComm::sendResult(CameraResultPacket packet)
{
    if (!status.initialized)
    {
        return false;
    }

    packet.checksum = 0;
    packet.checksum = cameraResultChecksum(packet);

    const esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&packet),
        sizeof(packet)
    );

    if (result == ESP_OK)
    {
        status.resultsSent++;
        return true;
    }

    status.sendErrors++;
    return false;
}

bool CameraComm::takeTrigger(CameraTriggerPacket &trigger)
{
    if (!pendingTriggerReady)
    {
        return false;
    }

    trigger = pendingTrigger;
    pendingTriggerReady = false;
    return true;
}

void CameraComm::onReceiveStatic(
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

void CameraComm::onReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
)
{
    if (info == nullptr || data == nullptr || len < 4)
    {
        return;
    }

    uint16_t magic = 0;
    memcpy(&magic, data, sizeof(magic));

    if (magic == SAFESEAT_HUB_BEACON_MAGIC
        && len == static_cast<int>(sizeof(SafeSeatHubBeacon)))
    {
        SafeSeatHubBeacon beacon;
        memcpy(&beacon, data, sizeof(beacon));

        if (beacon.version != SAFESEAT_NOW_PROTOCOL_VERSION
            || beacon.packetSize != sizeof(SafeSeatHubBeacon)
            || beacon.checksum != safeSeatHubBeaconChecksum(beacon)
            || beacon.channel < SAFESEAT_ESPNOW_MIN_CHANNEL
            || beacon.channel > SAFESEAT_ESPNOW_MAX_CHANNEL)
        {
            return;
        }

        status.hubLocked = true;
        status.lastHubBeaconMillis = millis();
        status.hubBeaconsReceived++;
        memcpy(status.hubMac, info->src_addr, 6);

        // If not associated/associating with the SafeSeat AP, follow
        // the Hub beacon exactly as in Step 5.9.4.  If Wi-Fi owns the
        // channel, do not fight the station connection.
        if (!stationChannelManaged && status.channel != beacon.channel)
        {
            setChannel(beacon.channel);
        }

        nextScanChannel = beacon.channel;
        return;
    }

    if (magic == CAMERA_TRIGGER_MAGIC
        && len == static_cast<int>(sizeof(CameraTriggerPacket)))
    {
        CameraTriggerPacket trigger;
        memcpy(&trigger, data, sizeof(trigger));

        if (trigger.version != CAMERA_WIRE_VERSION
            || trigger.packetSize != sizeof(CameraTriggerPacket)
            || trigger.checksum != cameraTriggerChecksum(trigger)
            || trigger.requestId == 0)
        {
            return;
        }

        status.triggerPacketsReceived++;

        if (trigger.requestId == lastAcceptedTriggerId)
        {
            status.duplicateTriggersIgnored++;
            return;
        }

        lastAcceptedTriggerId = trigger.requestId;
        pendingTrigger = trigger;
        pendingTriggerReady = true;
    }
}
