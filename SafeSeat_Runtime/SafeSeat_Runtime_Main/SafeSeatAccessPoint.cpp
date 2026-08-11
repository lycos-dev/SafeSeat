#include "SafeSeatAccessPoint.h"

#include "NetworkConfig.h"

bool SafeSeatAccessPoint::begin()
{
    status = SafeSeatAccessPointStatus{};

    // AP+STA is intentional:
    //   AP  -> phone + ESP32-S3 camera join "SafeSeat"
    //   STA -> SafeSeat ESP-NOW transport continues using WIFI_IF_STA
    WiFi.mode(WIFI_AP_STA);

    delay(50);

    const IPAddress localIp(
        SAFESEAT_AP_IP_A,
        SAFESEAT_AP_IP_B,
        SAFESEAT_AP_IP_C,
        SAFESEAT_AP_IP_D
    );

    const IPAddress gateway = localIp;

    const IPAddress subnet(
        SAFESEAT_AP_SUBNET_A,
        SAFESEAT_AP_SUBNET_B,
        SAFESEAT_AP_SUBNET_C,
        SAFESEAT_AP_SUBNET_D
    );

    if (!WiFi.softAPConfig(localIp, gateway, subnet))
    {
        Serial.println("[NETWORK] ERROR: SafeSeat SoftAP IP configuration failed.");
        return false;
    }

    const bool started = WiFi.softAP(
        SAFESEAT_AP_SSID,
        SAFESEAT_AP_PASSWORD,
        SAFESEAT_AP_CHANNEL,
        SAFESEAT_AP_HIDDEN,
        SAFESEAT_AP_MAX_CLIENTS
    );

    if (!started)
    {
        Serial.println("[NETWORK] ERROR: SafeSeat SoftAP failed to start.");
        return false;
    }

    delay(100);

    status.initialized = true;
    status.running = true;
    status.channel = WiFi.channel();
    status.connectedClients = static_cast<uint8_t>(WiFi.softAPgetStationNum());
    status.ipAddress = WiFi.softAPIP();

    Serial.println("[NETWORK] SafeSeat local Wi-Fi started.");
    Serial.print("[NETWORK] SSID    : ");
    Serial.println(SAFESEAT_AP_SSID);
    Serial.print("[NETWORK] IP      : ");
    Serial.println(status.ipAddress);
    Serial.print("[NETWORK] Channel : ");
    Serial.println(status.channel);
    Serial.println("[NETWORK] Internet: not required / local network only");

    if (status.channel != SAFESEAT_AP_CHANNEL)
    {
        Serial.print("[NETWORK] WARNING: requested channel ");
        Serial.print(SAFESEAT_AP_CHANNEL);
        Serial.print(" but radio reports channel ");
        Serial.println(status.channel);
    }

    return true;
}

void SafeSeatAccessPoint::update()
{
    if (!status.initialized)
    {
        return;
    }

    const wifi_mode_t mode = WiFi.getMode();
    status.running =
        mode == WIFI_AP
        || mode == WIFI_AP_STA;

    status.channel = WiFi.channel();
    status.connectedClients = static_cast<uint8_t>(WiFi.softAPgetStationNum());
    status.ipAddress = WiFi.softAPIP();
}
