#include "PiezoSensor.h"
#include <math.h>
#include <string.h>

PiezoSensor::PiezoSensor() {}

bool PiezoSensor::begin() {
    Serial.println("[PIEZO] Initializing seatbelt respiration sensor...");
    analogReadResolution(PIEZO_ADC_RESOLUTION_BITS);
    pinMode(PIEZO_PIN, INPUT);
    reading=PiezoReading{};
    filterInitialized=false; filteredSignal=0.0f; baseline=0.0f; previousWave=0.0f;
    lastBreathTime=0; breathHistoryCount=0; breathHistoryWriteIndex=0;
    memset(breathTimes,0,sizeof(breathTimes));
    memset(respirationBuffer,0,sizeof(respirationBuffer));
    memset(modelSourceBuffer,0,sizeof(modelSourceBuffer));
    bufferWriteIndex=0; bufferCount=0; samplesSinceFeatureWindow=0;
    acquisitionStartMillis=millis(); lastSampleMillis=acquisitionStartMillis;
    reading.status=PiezoStatus::READY; reading.valid=true;
    Serial.println("[PIEZO] Sensor ready.");
    Serial.printf("[PIEZO] Runtime sampling target: %.1f Hz\n", PIEZO_SAMPLE_RATE_HZ);
    Serial.printf("[PIEZO] ML window: %u sec / %u samples\n", PIEZO_WINDOW_SECONDS, PIEZO_WINDOW_SAMPLES);
    Serial.printf("[PIEZO] ML stride: %u sec\n", PIEZO_STRIDE_SECONDS);
    return true;
}

void PiezoSensor::update() {
    unsigned long now=millis();
    if(now-lastSampleMillis<PIEZO_SAMPLE_INTERVAL_MS) return;
    lastSampleMillis += PIEZO_SAMPLE_INTERVAL_MS;
    if(now-lastSampleMillis>PIEZO_SAMPLE_INTERVAL_MS*4UL) lastSampleMillis=now;
    int rawADC=analogRead(PIEZO_PIN);
    processSample(rawADC,now);
    reading.sampleCount++;
    updateSamplingDiagnostics(now);
}

void PiezoSensor::processSample(int rawADC,unsigned long now) {
    reading.rawADC=rawADC;
    if(rawADC<PIEZO_ADC_MIN || rawADC>PIEZO_ADC_MAX) {
        reading.valid=false; reading.status=PiezoStatus::INVALID_READING; return;
    }
    reading.valid=true; reading.status=PiezoStatus::READY;
    float raw=(float)rawADC;
    if(!filterInitialized) {
        filteredSignal=raw; baseline=raw; previousWave=0.0f; filterInitialized=true;
    } else {
        filteredSignal += PIEZO_EMA_ALPHA*(raw-filteredSignal);
        baseline += PIEZO_BASELINE_ALPHA*(filteredSignal-baseline);
    }
    float wave=filteredSignal-baseline;
    reading.filteredSignal=filteredSignal; reading.baseline=baseline; reading.respirationWave=wave;
    updateAuxiliaryBreathing(wave,now);
    storeSample(wave,raw);
    previousWave=wave;
}

void PiezoSensor::updateAuxiliaryBreathing(float wave,unsigned long now) {
    reading.breathDetectedThisSample=false;
    bool current=wave>PIEZO_PEAK_THRESHOLD;
    bool previous=previousWave>PIEZO_PEAK_THRESHOLD;
    if(current && !previous && (lastBreathTime==0 || now-lastBreathTime>=PIEZO_BREATH_COOLDOWN_MS)) recordBreath(now);
    reading.noBreathDurationMs=(lastBreathTime==0)? now-acquisitionStartMillis : now-lastBreathTime;
    reading.noBreathTimerExceeded=reading.noBreathDurationMs>=PIEZO_APNEA_TIME_MS;
    reading.estimatedRespirationBPM=calculateRollingRespirationBPM();
}

void PiezoSensor::recordBreath(unsigned long now) {
    reading.breathDetectedThisSample=true; reading.totalBreaths++; reading.lastBreathMillis=now; lastBreathTime=now;
    breathTimes[breathHistoryWriteIndex]=now;
    breathHistoryWriteIndex=(breathHistoryWriteIndex+1)%BREATH_HISTORY_SIZE;
    if(breathHistoryCount<BREATH_HISTORY_SIZE) breathHistoryCount++;
}

float PiezoSensor::calculateRollingRespirationBPM() const {
    if(breathHistoryCount<2) return NAN;
    uint8_t oldest=(breathHistoryCount<BREATH_HISTORY_SIZE)?0:breathHistoryWriteIndex;
    uint8_t newest=(breathHistoryWriteIndex+BREATH_HISTORY_SIZE-1)%BREATH_HISTORY_SIZE;
    unsigned long a=breathTimes[oldest], b=breathTimes[newest];
    if(b<=a) return NAN;
    float minutes=(b-a)/60000.0f;
    return minutes>0.0f ? (breathHistoryCount-1)/minutes : NAN;
}

void PiezoSensor::storeSample(float wave,float raw) {
    respirationBuffer[bufferWriteIndex]=wave;
    modelSourceBuffer[bufferWriteIndex]=raw;
    bufferWriteIndex=(bufferWriteIndex+1)%PIEZO_WINDOW_SAMPLES;
    if(bufferCount<PIEZO_WINDOW_SAMPLES) bufferCount++;
    reading.windowSamplesAvailable=bufferCount;
    reading.fullWindowReady=(bufferCount==PIEZO_WINDOW_SAMPLES);
    if(bufferCount==PIEZO_WINDOW_SAMPLES && reading.sampleCount==PIEZO_WINDOW_SAMPLES-1) {
        reading.newFeatureWindowDue=true; samplesSinceFeatureWindow=0; return;
    }
    if(bufferCount==PIEZO_WINDOW_SAMPLES) {
        samplesSinceFeatureWindow++;
        if(samplesSinceFeatureWindow>=PIEZO_STRIDE_SAMPLES) {
            reading.newFeatureWindowDue=true; samplesSinceFeatureWindow=0;
        }
    }
}

bool PiezoSensor::copyWindow(const float source[],float destination[],uint16_t destinationSize) const {
    if(!destination || destinationSize<PIEZO_WINDOW_SAMPLES || bufferCount<PIEZO_WINDOW_SAMPLES) return false;
    uint16_t idx=bufferWriteIndex;
    for(uint16_t i=0;i<PIEZO_WINDOW_SAMPLES;i++) {
        destination[i]=source[idx]; idx=(idx+1)%PIEZO_WINDOW_SAMPLES;
    }
    return true;
}

bool PiezoSensor::copyRespirationWindow(float destination[],uint16_t n) const { return copyWindow(respirationBuffer,destination,n); }
bool PiezoSensor::copyModelSourceWindow(float destination[],uint16_t n) const { return copyWindow(modelSourceBuffer,destination,n); }

void PiezoSensor::updateSamplingDiagnostics(unsigned long now) {
    unsigned long elapsed=now-acquisitionStartMillis;
    reading.actualSamplingRateHz=elapsed? (reading.sampleCount*1000.0f)/(float)elapsed : 0.0f;
}
