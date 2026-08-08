#include "PiezoSignalProcessor.h"
#include <math.h>

static constexpr uint8_t SOS_N=4;
static const float SOS[SOS_N][5]={
    { 0.000151409786023f, 0.000302819572047f, 0.000151409786023f, -1.61388852266f, 0.657542544439f },
    { 1.0f, 2.0f, 1.0f, -1.78502263072f, 0.841481720709f },
    { 1.0f, -2.0f, 1.0f, -1.97521336867f, 0.975399902633f },
    { 1.0f, -2.0f, 1.0f, -1.99096429198f, 0.991125447532f },
};

void PiezoSignalProcessor::linearDetrend(const float in[],float out[],uint16_t n) const {
    double sx=0,sy=0,sxx=0,sxy=0;
    for(uint16_t i=0;i<n;i++){ double x=i,y=in[i]; sx+=x; sy+=y; sxx+=x*x; sxy+=x*y; }
    double N=n,den=N*sxx-sx*sx,slope=0,intercept=sy/N;
    if(fabs(den)>1e-18){ slope=(N*sxy-sx*sy)/den; intercept=(sy-slope*sx)/N; }
    for(uint16_t i=0;i<n;i++) out[i]=(float)(in[i]-(slope*i+intercept));
}

void PiezoSignalProcessor::filterForward(float x[],uint16_t n) const {
    State st[SOS_N];
    for(uint16_t i=0;i<n;i++){
        float v=x[i];
        for(uint8_t s=0;s<SOS_N;s++){
            float y=SOS[s][0]*v+st[s].z1;
            st[s].z1=SOS[s][1]*v-SOS[s][3]*y+st[s].z2;
            st[s].z2=SOS[s][2]*v-SOS[s][4]*y;
            v=y;
        }
        x[i]=v;
    }
}

void PiezoSignalProcessor::reverse(float x[],uint16_t n) const {
    for(uint16_t i=0;i<n/2;i++){ float t=x[i]; x[i]=x[n-1-i]; x[n-1-i]=t; }
}

void PiezoSignalProcessor::quality(const float raw[],const float aligned[],uint16_t n,PiezoSignalQuality &q) const {
    q=PiezoSignalQuality{}; uint16_t rails=0; double sum=0; float mn=aligned[0],mx=aligned[0];
    for(uint16_t i=0;i<n;i++){
        if(raw[i]<=PIEZO_ADC_MIN+PIEZO_ADC_RAIL_MARGIN || raw[i]>=PIEZO_ADC_MAX-PIEZO_ADC_RAIL_MARGIN) rails++;
        float v=aligned[i]; sum+=v; if(v<mn)mn=v; if(v>mx)mx=v;
    }
    float mean=(float)(sum/n); double var=0;
    for(uint16_t i=0;i<n;i++){ double d=aligned[i]-mean; var+=d*d; } var/=n;
    q.railFraction=(float)rails/n; q.alignedMean=mean; q.alignedStd=(float)sqrt(var); q.alignedMin=mn; q.alignedMax=mx;
    q.excessiveRailContact=q.railFraction>PIEZO_MAX_RAIL_FRACTION;
    q.effectivelyFlat=q.alignedStd<PIEZO_MIN_ALIGNED_STD;
    q.valid=!q.excessiveRailContact && !q.effectivelyFlat;
}

bool PiezoSignalProcessor::alignWindow(const float raw[],float aligned[],uint16_t n,PiezoSignalQuality &q) const {
    if(!raw||!aligned||n!=PIEZO_WINDOW_SAMPLES) return false;
    linearDetrend(raw,aligned,n);
    filterForward(aligned,n); reverse(aligned,n); filterForward(aligned,n); reverse(aligned,n);
    quality(raw,aligned,n,q); return true;
}
