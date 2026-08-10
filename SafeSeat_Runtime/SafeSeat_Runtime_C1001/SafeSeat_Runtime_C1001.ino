#include <Arduino.h>

#include "Config.h"
#include "C1001.h"
#include "C1001ML.h"
#include "C1001Comm.h"

C1001Sensor c1001;
C1001ML c1001ML;
C1001Comm c1001Comm;

bool c1001Initialized = false;
bool commInitialized = false;
unsigned long lastPrintMillis = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" SafeSeat Remote C1001 Node");
    Serial.println(" Step 5.8 - M1A C1001 + ESP-NOW");
    Serial.println("==========================================");

    c1001Initialized = c1001.begin();
    c1001ML.begin();

    commInitialized = c1001Comm.begin();

    Serial.println();
    Serial.print("C1001 acquisition : ");
    Serial.println(c1001Initialized ? "READY" : "FAILED");
    Serial.print("ESP-NOW transport : ");
    Serial.println(commInitialized ? "READY" : "FAILED");
    Serial.println();
}

void loop()
{
    if (c1001Initialized)
    {
        c1001.update();
    }

    const C1001Reading &reading = c1001.getReading();

    c1001ML.update(reading);
    const C1001MLReading &model = c1001ML.getReading();

    if (commInitialized)
    {
        c1001Comm.update(reading, model);
    }

    const unsigned long now = millis();
    if (now - lastPrintMillis < C1001_NODE_PRINT_INTERVAL_MS)
    {
        delay(1);
        return;
    }

    lastPrintMillis = now;

    Serial.println("==========================================");
    Serial.println(" SAFESEAT C1001 NODE");
    Serial.println("==========================================");

    Serial.print("Sensor         : ");
    Serial.println(c1001Initialized ? "READY" : "FAILED");
    Serial.print("Status         : ");
    Serial.println(c1001.getStatusText());
    Serial.print("Presence       : ");
    Serial.println(reading.present ? "YES" : "NO");
    Serial.print("MoveRange      : ");
    Serial.println(reading.moveRange);
    Serial.print("Raw RR / HR    : ");
    Serial.print(reading.rawRespiration);
    Serial.print(" / ");
    Serial.println(reading.rawHeartRate);

    if (reading.trustedVitalsAvailable)
    {
        Serial.print("Filtered RR    : ");
        Serial.print(reading.filteredRespiration, 1);
        Serial.println(" BPM");
        Serial.print("Filtered HR    : ");
        Serial.print(reading.filteredHeartRate, 1);
        Serial.println(" BPM");
    }
    else
    {
        Serial.println("Filtered vitals: not ready");
    }

    Serial.println();
    Serial.print("ML status      : ");
    Serial.println(c1001ML.getStatusText());
    Serial.print("ML window      : ");
    Serial.print(model.windowSamplesCollected);
    Serial.print(" / ");
    Serial.println(model.windowSamplesRequired);
    Serial.print("ML windows     : ");
    Serial.println(model.windowsEvaluated);

    if (model.valid)
    {
        Serial.print("IF decision    : ");
        Serial.print(model.isolationForestDecision, 6);
        Serial.println(model.isolationForestAnomaly ? " [ANOMALY]" : " [NORMAL]");
        Serial.print("SVM decision   : ");
        Serial.print(model.oneClassSVMDecision, 6);
        Serial.println(model.oneClassSVMAnomaly ? " [ANOMALY]" : " [NORMAL]");
    }

    Serial.println();
    Serial.print("ESP-NOW link   : ");
    Serial.println(c1001Comm.isHubLocked() ? "LOCKED" : "SCANNING");
    Serial.print("Wi-Fi channel  : ");
    Serial.println(c1001Comm.getChannel());
    Serial.print("Hub beacons    : ");
    Serial.println(c1001Comm.getHubBeaconsReceived());
    Serial.print("Beacon age     : ");
    Serial.print(c1001Comm.getBeaconAgeMillis());
    Serial.println(" ms");
    Serial.print("Packets sent   : ");
    Serial.println(c1001Comm.getPacketsSent());
    Serial.print("Send errors    : ");
    Serial.println(c1001Comm.getSendErrors());
    Serial.println();
}
