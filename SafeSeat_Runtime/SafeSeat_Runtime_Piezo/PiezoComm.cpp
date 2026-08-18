#include "PiezoComm.h"

#include <esp_wifi.h>

#include "Config.h"

PiezoComm* PiezoComm::activeInstance = nullptr;

const uint8_t PiezoComm::BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

bool PiezoComm::begin()
{
    if (initialized)
    {
        return true;
    }

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

    currentChannel = SAFESEAT_ESPNOW_DEFAULT_CHANNEL;

    if (
        WiFi.setChannel(
            currentChannel,
            WIFI_SECOND_CHAN_NONE
        )
        !=
        ESP_OK
    )
    {
        return false;
    }

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    activeInstance = this;

    if (
        esp_now_register_recv_cb(
            &PiezoComm::onReceiveStatic
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

bool PiezoComm::ensureBroadcastPeer()
{
    if (esp_now_is_peer_exist(BROADCAST_MAC))
    {
        return true;
    }

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0; // use whatever channel the Piezo radio is currently on
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t result = esp_now_add_peer(&peer);

    return
        result == ESP_OK
        ||
        result == ESP_ERR_ESPNOW_EXIST;
}

unsigned long PiezoComm::getBeaconAgeMillis() const
{
    if (lastHubBeaconMillis == 0)
    {
        return 0;
    }

    return millis() - lastHubBeaconMillis;
}

void PiezoComm::maintainHubChannel()
{
    const unsigned long now = millis();

    if (
        hubLocked
        &&
        lastHubBeaconMillis != 0
        &&
        now - lastHubBeaconMillis
        <=
        SAFESEAT_ESPNOW_HUB_BEACON_TIMEOUT_MS
    )
    {
        return;
    }

    hubLocked = false;

    if (
        now - lastChannelHopMillis
        >=
        SAFESEAT_ESPNOW_CHANNEL_DWELL_MS
    )
    {
        lastChannelHopMillis = now;
        hopToNextChannel();
    }
}

void PiezoComm::hopToNextChannel()
{
    if (
        nextScanChannel < SAFESEAT_ESPNOW_MIN_CHANNEL
        ||
        nextScanChannel > SAFESEAT_ESPNOW_MAX_CHANNEL
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
        WiFi.setChannel(
            nextScanChannel,
            WIFI_SECOND_CHAN_NONE
        )
        ==
        ESP_OK
    )
    {
        currentChannel = nextScanChannel;
    }
}

void PiezoComm::onReceiveStatic(
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

void PiezoComm::onReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
)
{
    (void)info;

    // Keep the Wi-Fi callback tiny.  We only validate the small
    // hub beacon and update channel-lock timestamps here.
    if (
        data == nullptr
        ||
        len != static_cast<int>(sizeof(SafeSeatHubBeacon))
    )
    {
        return;
    }

    SafeSeatHubBeacon beacon;
    memcpy(
        &beacon,
        data,
        sizeof(beacon)
    );

    if (
        beacon.magic != SAFESEAT_HUB_BEACON_MAGIC
        ||
        beacon.version != SAFESEAT_NOW_PROTOCOL_VERSION
        ||
        beacon.packetSize != sizeof(SafeSeatHubBeacon)
        ||
        beacon.checksum != safeSeatHubBeaconChecksum(beacon)
        ||
        beacon.channel < SAFESEAT_ESPNOW_MIN_CHANNEL
        ||
        beacon.channel > SAFESEAT_ESPNOW_MAX_CHANNEL
    )
    {
        return;
    }

    // A beacon can only be received while our radio is already on
    // its channel.  Lock immediately and stop channel hopping.
    currentChannel = beacon.channel;
    nextScanChannel = beacon.channel;
    lastHubBeaconMillis = millis();
    hubBeaconsReceived++;
    hubLocked = true;
}

void PiezoComm::update(
    const PiezoReading &reading
)
{
    if (!initialized)
    {
        return;
    }

    maintainHubChannel();

    const unsigned long now =
        millis();

    if (
        now - lastSendMillis
        <
        PIEZO_COMM_TX_INTERVAL_MS
    )
    {
        return;
    }

    lastSendMillis =
        now;

    PiezoWirePacket packet;

    packet.sequence =
        ++sequence;

    packet.senderMillis =
        now;

    packet.flags =
        0;

    if (reading.valid)
    {
        packet.flags |=
            PIEZO_FLAG_SENSOR_VALID;
    }

    if (reading.signalUsable)
    {
        packet.flags |=
            PIEZO_FLAG_SIGNAL_USABLE;
    }

    if (reading.breathTrackingReady)
    {
        packet.flags |=
            PIEZO_FLAG_BREATH_TRACKING_READY;
    }

    if (
        isfinite(
            reading.estimatedRespirationBPM
        )
    )
    {
        packet.flags |=
            PIEZO_FLAG_RR_VALID;

        packet.respirationBPM =
            reading.estimatedRespirationBPM;
    }
    else
    {
        packet.respirationBPM =
            NAN;
    }

    if (reading.noBreathTimerExceeded)
    {
        packet.flags |=
            PIEZO_FLAG_NO_BREATH_TIMER;
    }

    if (reading.breathDetectedRecently)
    {
        packet.flags |=
            PIEZO_FLAG_BREATH_DETECTED_RECENT;
    }

    packet.respirationWave =
        reading.respirationWave;

    packet.actualSamplingRateHz =
        reading.actualSamplingRateHz;

    packet.totalBreaths =
        static_cast<uint32_t>(
            reading.totalBreaths
        );

    packet.noBreathDurationMs =
        static_cast<uint32_t>(
            reading.noBreathDurationMs
        );

    packet.sampleCount =
        static_cast<uint32_t>(
            reading.sampleCount
        );

    packet.checksum =
        0;

    packet.checksum =
        piezoPacketChecksum(
            packet
        );

    const esp_err_t result =
        esp_now_send(
            BROADCAST_MAC,
            reinterpret_cast<const uint8_t *>(&packet),
            sizeof(packet)
        );

    if (
        result
        ==
        ESP_OK
    )
    {
        packetsSent++;
    }
    else
    {
        sendErrors++;
    }
}
