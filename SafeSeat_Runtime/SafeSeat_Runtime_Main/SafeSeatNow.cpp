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

    // ESP-NOW needs an active Wi-Fi interface.  The Main Hub uses
    // STA because the future frontend/server Wi-Fi path will also
    // use the station interface.  If that Wi-Fi connection later
    // changes channel, our broadcast peer uses channel 0 (current
    // channel) and the remote Piezo scanner can rediscover it.
    WiFi.mode(WIFI_STA);

    unsigned long waitStart = millis();
    while (
        !WiFi.STA.started()
        &&
        millis() - waitStart < 2000UL
    )
    {
        delay(10);
    }

    if (!WiFi.STA.started())
    {
        return false;
    }

    // No infrastructure Wi-Fi is configured in Step 5.7.3 yet.
    // Use a deterministic local channel for now.  When the future
    // frontend Wi-Fi joins an AP, the AP's channel becomes the
    // Main Hub channel and the Piezo will automatically rescan.
    if (WiFi.status() != WL_CONNECTED)
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

    if (
        esp_now_register_recv_cb(
            &SafeSeatNow::onReceiveStatic
        )
        !=
        ESP_OK
    )
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
    pendingPiezoReady = false;

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
    peer.channel = 0; // always use the local device's current channel
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t result = esp_now_add_peer(&peer);

    return
        result == ESP_OK
        ||
        result == ESP_ERR_ESPNOW_EXIST;
}

void SafeSeatNow::update()
{
    if (!status.initialized)
    {
        return;
    }

    status.channel = WiFi.channel();

    const unsigned long now = millis();

    if (
        now - lastBeaconMillis
        >=
        SAFESEAT_ESPNOW_HUB_BEACON_INTERVAL_MS
    )
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
        activeInstance->onReceive(
            info,
            data,
            len
        );
    }
}

void SafeSeatNow::onReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
)
{
    // ESP-NOW callbacks execute in a high-priority Wi-Fi task.
    // Do only the tiny packet copy here; parsing/Fusion happens
    // later from the normal Arduino loop.
    if (
        info == nullptr
        ||
        data == nullptr
        ||
        len != static_cast<int>(sizeof(PiezoWirePacket))
    )
    {
        return;
    }

    const PiezoWirePacket *packet =
        reinterpret_cast<const PiezoWirePacket *>(data);

    if (
        packet->magic != PIEZO_WIRE_MAGIC
        ||
        packet->version != PIEZO_WIRE_VERSION
        ||
        packet->packetSize != sizeof(PiezoWirePacket)
    )
    {
        return;
    }

    memcpy(
        &pendingPiezoPacket,
        data,
        sizeof(PiezoWirePacket)
    );

    memcpy(
        pendingPiezoMac,
        info->src_addr,
        6
    );

    // Written last so the normal loop only observes complete data.
    pendingPiezoReady = true;
    status.piezoPacketsQueued++;
}

bool SafeSeatNow::takeLatestPiezoPacket(
    PiezoWirePacket &packet,
    uint8_t sourceMac[6]
)
{
    if (!pendingPiezoReady)
    {
        return false;
    }

    // The Piezo transmits at only 2 Hz.  Copying the 40-byte latest
    // packet in the loop is intentionally lightweight.  CRC is
    // validated by PiezoComm after this copy, so a pathological
    // concurrent overwrite is rejected rather than fused.
    packet = pendingPiezoPacket;
    memcpy(sourceMac, pendingPiezoMac, 6);
    pendingPiezoReady = false;

    return true;
}
