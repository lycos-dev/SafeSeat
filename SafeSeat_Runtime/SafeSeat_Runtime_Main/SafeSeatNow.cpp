#include "SafeSeatNow.h"

#include <esp_wifi.h>

#include "Config.h"

SafeSeatNow* SafeSeatNow::activeInstance = nullptr;

const uint8_t SafeSeatNow::BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

SafeSeatNow& SafeSeatNow::instance()
{
    static SafeSeatNow transport;
    return transport;
}

bool SafeSeatNow::begin()
{
    if (status.initialized)
    {
        return true;
    }

    // Step 5.9.6: preserve the Main Hub SoftAP while keeping the
    // station interface available for ESP-NOW. Re-entering WIFI_STA
    // here would disable the SafeSeat AP.
    if (WiFi.getMode() != WIFI_AP_STA)
    {
        WiFi.mode(WIFI_AP_STA);
    }

    const unsigned long waitStart = millis();
    while (!WiFi.STA.started() && millis() - waitStart < 2000UL)
    {
        delay(10);
    }

    if (!WiFi.STA.started())
    {
        return false;
    }

    // If the SafeSeat SoftAP is running, it is the channel authority.
    // Do not retune the radio underneath it. If the AP failed to start,
    // retain the old standalone ESP-NOW fallback channel.
    const bool softApRunning = WiFi.softAPSSID().length() > 0;

    if (!softApRunning && WiFi.status() != WL_CONNECTED)
    {
        WiFi.setChannel(
            SAFESEAT_ESPNOW_DEFAULT_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        );
    }

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    activeInstance = this;

    if (esp_now_register_recv_cb(&SafeSeatNow::onReceiveStatic) != ESP_OK)
    {
        return false;
    }

    if (!ensureBroadcastPeer())
    {
        return false;
    }

    status = SafeSeatNowStatus{};
    status.initialized = true;
    status.channel = WiFi.channel();
    lastBeaconMillis = 0;
    pendingC1001Ready = false;
    pendingCameraStatusReady = false;
    pendingCameraResultReady = false;

    return true;
}

bool SafeSeatNow::ensureBroadcastPeer()
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

void SafeSeatNow::update()
{
    if (!status.initialized)
    {
        return;
    }

    status.channel = WiFi.channel();

    const unsigned long now = millis();
    if (now - lastBeaconMillis >= SAFESEAT_ESPNOW_HUB_BEACON_INTERVAL_MS)
    {
        lastBeaconMillis = now;
        sendHubBeacon();
    }
}

void SafeSeatNow::sendHubBeacon()
{
    SafeSeatHubBeacon beacon;
    beacon.channel = WiFi.channel();
    beacon.checksum = 0;
    beacon.checksum = safeSeatHubBeaconChecksum(beacon);

    const esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&beacon),
        sizeof(beacon)
    );

    if (result == ESP_OK)
    {
        status.hubBeaconsSent++;
    }
    else
    {
        status.hubBeaconSendErrors++;
    }
}

void SafeSeatNow::onReceiveStatic(
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

void SafeSeatNow::onReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
)
{
    if (info == nullptr || data == nullptr || len < 4)
    {
        return;
    }

    // The first 16 bits are a sensor-specific magic value for all
    // current SafeSeat node packets. Copy it without assuming
    // alignment inside the Wi-Fi callback buffer.
    uint16_t magic = 0;
    memcpy(&magic, data, sizeof(magic));


    if (
        magic == C1001_WIRE_MAGIC
        && len == static_cast<int>(sizeof(C1001WirePacket))
    )
    {
        C1001WirePacket packet;
        memcpy(&packet, data, sizeof(packet));

        if (
            packet.version == C1001_WIRE_VERSION
            && packet.packetSize == sizeof(C1001WirePacket)
        )
        {
            pendingC1001Packet = packet;
            memcpy(pendingC1001Mac, info->src_addr, 6);
            pendingC1001Ready = true;
            status.c1001PacketsQueued++;
            return;
        }
    }


    if (
        magic == CAMERA_STATUS_MAGIC
        && len == static_cast<int>(sizeof(CameraStatusPacket))
    )
    {
        CameraStatusPacket packet;
        memcpy(&packet, data, sizeof(packet));

        if (
            packet.version == CAMERA_WIRE_VERSION
            && packet.packetSize == sizeof(CameraStatusPacket)
        )
        {
            pendingCameraStatus = packet;
            memcpy(pendingCameraStatusMac, info->src_addr, 6);
            pendingCameraStatusReady = true;
            status.cameraStatusPacketsQueued++;
            return;
        }
    }

    if (
        magic == CAMERA_RESULT_MAGIC
        && len == static_cast<int>(sizeof(CameraResultPacket))
    )
    {
        CameraResultPacket packet;
        memcpy(&packet, data, sizeof(packet));

        if (
            packet.version == CAMERA_WIRE_VERSION
            && packet.packetSize == sizeof(CameraResultPacket)
        )
        {
            pendingCameraResult = packet;
            memcpy(pendingCameraResultMac, info->src_addr, 6);
            pendingCameraResultReady = true;
            status.cameraResultPacketsQueued++;
            return;
        }
    }

    status.unknownPacketsIgnored++;
}


bool SafeSeatNow::takeLatestC1001Packet(
    C1001WirePacket &packet,
    uint8_t sourceMac[6]
)
{
    if (!pendingC1001Ready)
    {
        return false;
    }

    packet = pendingC1001Packet;
    memcpy(sourceMac, pendingC1001Mac, 6);
    pendingC1001Ready = false;
    return true;
}


bool SafeSeatNow::takeLatestCameraStatus(
    CameraStatusPacket &packet,
    uint8_t sourceMac[6]
)
{
    if (!pendingCameraStatusReady)
    {
        return false;
    }

    packet = pendingCameraStatus;
    memcpy(sourceMac, pendingCameraStatusMac, 6);
    pendingCameraStatusReady = false;
    return true;
}

bool SafeSeatNow::takeLatestCameraResult(
    CameraResultPacket &packet,
    uint8_t sourceMac[6]
)
{
    if (!pendingCameraResultReady)
    {
        return false;
    }

    packet = pendingCameraResult;
    memcpy(sourceMac, pendingCameraResultMac, 6);
    pendingCameraResultReady = false;
    return true;
}

bool SafeSeatNow::sendCameraTrigger(
    const CameraTriggerPacket &packet
)
{
    if (!status.initialized)
    {
        return false;
    }

    const esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&packet),
        sizeof(packet)
    );

    if (result == ESP_OK)
    {
        status.cameraTriggersSent++;
        return true;
    }

    status.cameraTriggerSendErrors++;
    return false;
}
