#include <Arduino.h>
#include "Config.h"
#include "PiezoSensor.h"
#include "PiezoSignalProcessor.h"
#include "PiezoFeatureExtractor.h"
#include "PiezoInference.h"

PiezoSensor piezo;
PiezoSignalProcessor signalProcessor;
PiezoFeatureExtractor featureExtractor;
PiezoInference piezoInference;

PiezoFeatures latestFeatures;
PiezoInferenceResult latestInference;
PiezoSignalQuality latestSignalQuality;
bool featureVectorReady=false, inferenceReady=false, signalWindowAligned=false;
unsigned long featureWindowCount=0, lastReportMillis=0;
float modelSourceWindow[PIEZO_WINDOW_SAMPLES];
float alignedWindow[PIEZO_WINDOW_SAMPLES];
float normalizedWindow[PIEZO_WINDOW_SAMPLES];

void processModelWindow(){
    const PiezoReading &p=piezo.getReading();
    if(!p.newFeatureWindowDue) return;
    featureVectorReady=false; inferenceReady=false; signalWindowAligned=false; latestSignalQuality=PiezoSignalQuality{};
    if(!piezo.copyModelSourceWindow(modelSourceWindow,PIEZO_WINDOW_SAMPLES)){ piezo.acknowledgeFeatureWindow(); return; }
    if(!signalProcessor.alignWindow(modelSourceWindow,alignedWindow,PIEZO_WINDOW_SAMPLES,latestSignalQuality)){ piezo.acknowledgeFeatureWindow(); return; }
    signalWindowAligned=true;
    if(!latestSignalQuality.valid){ piezo.acknowledgeFeatureWindow(); return; }
    if(!featureExtractor.robustNormalizeWindow(alignedWindow,normalizedWindow,PIEZO_WINDOW_SAMPLES)){ piezo.acknowledgeFeatureWindow(); return; }
    if(!featureExtractor.computeFeatures(normalizedWindow,PIEZO_WINDOW_SAMPLES,latestFeatures)){ piezo.acknowledgeFeatureWindow(); return; }
    featureVectorReady=true; featureWindowCount++;
    inferenceReady=piezoInference.predict(latestFeatures,latestInference);
    piezo.acknowledgeFeatureWindow();
}

void setup(){
    Serial.begin(115200); delay(1000);
    Serial.println("\n==========================================");
    Serial.println(" SafeSeat Seatbelt Runtime");
    Serial.println(" Stage 4 - Piezo Signal Alignment");
    Serial.println("==========================================");
    bool ok=piezo.begin(); Serial.println(ok?"[PIEZO] Acquisition ready.":"[PIEZO] ERROR.");
    Serial.println("[ML] raw ADC -> detrend -> 0.05-1 Hz zero-phase bandpass");
    Serial.println("     -> median/MAD -> 16 features -> IF + OCSVM\n");
}

void loop(){
    piezo.update(); processModelWindow();
    unsigned long now=millis(); if(now-lastReportMillis<PIEZO_SERIAL_REPORT_INTERVAL_MS) return; lastReportMillis=now;
    const PiezoReading &p=piezo.getReading();
    Serial.println("\n==========================================");
    Serial.println(" SAFESEAT PIEZO ESP32 - STAGE 4");
    Serial.println("==========================================");
    Serial.printf("Sample rate       : %.2f Hz\n",p.actualSamplingRateHz);
    Serial.printf("Samples           : %lu\n",p.sampleCount);
    Serial.printf("Raw ADC           : %d\n",p.rawADC);
    Serial.printf("Aux resp waveform : %.2f\n",p.respirationWave);
    if(isfinite(p.estimatedRespirationBPM)) Serial.printf("Peak-based RR     : %.1f BPM\n",p.estimatedRespirationBPM); else Serial.println("Peak-based RR     : not ready");
    Serial.printf("No-breath time    : %.1f sec\n",p.noBreathDurationMs/1000.0f);
    Serial.printf("30s window        : %u / %u\n",p.windowSamplesAvailable,PIEZO_WINDOW_SAMPLES);
    Serial.println("------------------------------------------");
    if(signalWindowAligned){
        Serial.printf("Signal quality    : %s\n",latestSignalQuality.valid?"VALID":"REJECTED");
        Serial.printf("ADC rail fraction : %.2f %%\n",latestSignalQuality.railFraction*100.0f);
        Serial.printf("Aligned std       : %.4f\n",latestSignalQuality.alignedStd);
        Serial.printf("Aligned range     : %.4f\n",latestSignalQuality.alignedMax-latestSignalQuality.alignedMin);
        Serial.printf("Latest aligned    : %.4f\n",alignedWindow[PIEZO_WINDOW_SAMPLES-1]);
        if(latestSignalQuality.excessiveRailContact) Serial.println("Reject reason     : ADC clipping/rail contact");
        if(latestSignalQuality.effectivelyFlat) Serial.println("Reject reason     : aligned signal too flat");
    } else Serial.println("Signal alignment  : waiting for first 30s window");
    Serial.println("------------------------------------------");
    Serial.printf("Feature windows   : %lu\n",featureWindowCount);
    if(featureVectorReady){
        Serial.printf("Spectral RR       : %.2f BPM\n",latestFeatures.respirationBPM);
        Serial.printf("Spectral entropy  : %.6f\n",latestFeatures.spectralEntropy);
        Serial.printf("Autocorr peak     : %.6f\n",latestFeatures.autocorrelationPeak);
    }
    if(inferenceReady && latestInference.valid){
        Serial.printf("IsolationForest   : %.6f -> %s\n",latestInference.isolationForestDecision,latestInference.isolationForestAnomaly?"ANOMALY":"NORMAL");
        Serial.printf("One-Class SVM     : %.6f -> %s\n",latestInference.oneClassSVMDecision,latestInference.oneClassSVMAnomaly?"ANOMALY":"NORMAL");
        Serial.printf("BOTH models       : %s\n",latestInference.bothModelsAnomaly?"ANOMALY":"NORMAL");
    } else if(signalWindowAligned && !latestSignalQuality.valid) Serial.println("Inference         : SKIPPED - poor signal quality");
    else Serial.println("Inference         : waiting");
    Serial.println("==========================================");
}
