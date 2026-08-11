#include "SafeSeatWiFi.h"

#include "Config.h"

void SafeSeatWiFi::begin()
{
    status = SafeSeatWiFiStatus{};
    status.enabled = SAFESEAT_WIFI_STA_ENABLED;

    // CameraComm owns Wi-Fi STA radio initialization because ESP-NOW
    // shares that same interface.  This module only manages association.

    if (!status.enabled)
    {
        Serial.println("[WiFi] SafeSeat STA join disabled in Config.h");
        return;
    }

    Serial.print("[WiFi] Final target SSID: ");
    Serial.println(SAFESEAT_WIFI_SSID);
    Serial.println("[WiFi] Main Hub SoftAP is added in Step 5.9.6.");
}

void SafeSeatWiFi::startAttempt()
{
    if (!status.enabled || status.connecting || status.connected)
    {
        return;
    }

    Serial.println();
    Serial.print("[WiFi] Joining SafeSeat AP: ");
    Serial.println(SAFESEAT_WIFI_SSID);

    // Keep ESP-NOW initialized.  WiFi.begin() only asks the existing
    // STA interface to associate with the Main Hub's AP.
    WiFi.begin(SAFESEAT_WIFI_SSID, SAFESEAT_WIFI_PASSWORD);

    status.connecting = true;
    status.attempts++;
    status.lastAttemptMillis = millis();
    attemptStartMillis = status.lastAttemptMillis;
    firstAttemptPending = false;
}

void SafeSeatWiFi::stopAttempt()
{
    // Disconnect STA association without powering down the Wi-Fi radio;
    // ESP-NOW continues using the same STA interface.
    WiFi.disconnect(false, false);
    status.connecting = false;
}

void SafeSeatWiFi::update()
{
    if (!status.enabled)
    {
        return;
    }

    const unsigned long now = millis();
    const wl_status_t wifiStatus = WiFi.status();

    if (wifiStatus == WL_CONNECTED)
    {
        if (!status.connected)
        {
            status.connected = true;
            status.connecting = false;
            status.successfulConnections++;
            status.connectedSinceMillis = now;

            Serial.println();
            Serial.println("[WiFi] Connected to SafeSeat AP.");
            Serial.print("[WiFi] IP      : "); Serial.println(WiFi.localIP());
            Serial.print("[WiFi] Channel : "); Serial.println(WiFi.channel());
            Serial.print("[WiFi] RSSI    : "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
        }
        return;
    }

    if (status.connected)
    {
        status.connected = false;
        status.connectedSinceMillis = 0;
        Serial.println("[WiFi] SafeSeat AP connection lost; ESP-NOW discovery remains active.");
    }

    if (status.connecting)
    {
        if (now - attemptStartMillis >= SAFESEAT_WIFI_CONNECT_TIMEOUT_MS)
        {
            Serial.println("[WiFi] SafeSeat AP not available yet; returning radio to ESP-NOW discovery.");
            stopAttempt();
        }
        return;
    }

    if (firstAttemptPending
        || now - status.lastAttemptMillis >= SAFESEAT_WIFI_RETRY_INTERVAL_MS)
    {
        startAttempt();
    }
}
