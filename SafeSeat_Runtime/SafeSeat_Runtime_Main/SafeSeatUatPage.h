#pragma once

#include <Arduino.h>

// ============================================================
// SAFESEAT MAINTENANCE MONITOR
//
// Served at /uat (and /maintenance) for researcher/maintenance use.
// Read-only. No warning/emergency injection is exposed here.
// IMPORTANT: manual refresh is the default. Optional live mode uses
// one consolidated /api/v1/status request every 5 seconds.
// ============================================================

static const char SAFESEAT_UAT_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>SafeSeat Maintenance Monitor</title>
<style>
:root{color-scheme:dark;--bg:#08120f;--panel:#0f1d19;--panel2:#0b1714;--line:#244138;--text:#f4fbf8;--muted:#9db5ad;--ok:#54d68b;--warn:#f1bf57;--bad:#ff6c6c;--accent:#5ed5a0}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#07100e,#0a1512 45%,#08110f);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}header{position:sticky;top:0;z-index:5;background:rgba(8,18,15,.94);backdrop-filter:blur(8px);border-bottom:1px solid var(--line);padding:14px 16px}.title{font-size:19px;font-weight:850;display:flex;gap:8px;align-items:center;flex-wrap:wrap}.badge{font-size:10px;letter-spacing:.06em;padding:4px 7px;border-radius:999px;background:#173128;color:#bff4d8}.sub{font-size:11px;color:var(--muted);margin-top:4px}.wrap{max-width:1050px;margin:auto;padding:14px}.controls{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}.btn{border:1px solid #315248;background:#16342a;color:#effff7;border-radius:10px;padding:10px 13px;font-weight:750}.btn.secondary{background:#111f1b}.btn.active{background:#25523f;border-color:#4b9b76}.statusline{font-size:11px;color:var(--muted);padding:5px 1px 11px}.top{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px;margin-bottom:10px}.mini,.card{background:rgba(15,29,25,.94);border:1px solid var(--line);border-radius:14px}.mini{padding:11px}.k{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}.mini .v{font-size:19px;font-weight:850;margin-top:4px}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.card{padding:13px}.card h2{font-size:14px;margin:0 0 9px}.kv{display:grid;grid-template-columns:1.35fr 1fr;gap:8px;padding:5px 0;border-bottom:1px solid #182c26}.kv:last-child{border:0}.kv .v{text-align:right;font-variant-numeric:tabular-nums;font-size:12px}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}.fsr{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:9px}.fsrCell{background:var(--panel2);border:1px solid var(--line);border-radius:9px;padding:7px}.fsrCell strong{display:block;font-size:15px}.raw{display:none;white-space:pre-wrap;word-break:break-word;background:#050b09;border:1px solid var(--line);border-radius:10px;padding:9px;max-height:300px;overflow:auto;font:10px ui-monospace,SFMono-Regular,Consolas,monospace}.footer{font-size:10px;color:var(--muted);text-align:center;padding:16px 0}.full{grid-column:1/-1}@media(max-width:760px){.grid{grid-template-columns:1fr}.top{grid-template-columns:1fr 1fr}}@media(max-width:420px){.mini .v{font-size:16px}.wrap{padding:10px}.card{padding:11px}}
</style>
</head>
<body>
<header><div class="title">SafeSeat Maintenance <span class="badge">READ-ONLY</span></div><div class="sub">Technical service view. Live refresh is intentionally off by default.</div></header>
<div class="wrap">
  <div class="controls">
    <button class="btn" onclick="refreshNow()">Refresh snapshot</button>
    <button class="btn secondary" id="liveBtn" onclick="toggleLive()">Live Maintenance: OFF</button>
    <button class="btn secondary" onclick="toggleRaw()">Raw JSON</button>
  </div>
  <div class="statusline" id="msg">Press Refresh snapshot, or enable Live Maintenance for a 5-second refresh.</div>
  <div class="top">
    <div class="mini"><div class="k">Hub</div><div class="v" id="hub">--</div></div>
    <div class="mini"><div class="k">Fusion</div><div class="v" id="fusion">--</div></div>
    <div class="mini"><div class="k">Occupancy</div><div class="v" id="occupancy">--</div></div>
    <div class="mini"><div class="k">Clients</div><div class="v" id="clients">--</div></div>
  </div>
  <div class="grid">
    <section class="card"><h2>C1001</h2><div class="kv"><span class="k">Link</span><span class="v" id="cLink">--</span></div><div class="kv"><span class="k">Presence</span><span class="v" id="cPres">--</span></div><div class="kv"><span class="k">Heart rate</span><span class="v" id="cHr">--</span></div><div class="kv"><span class="k">Respiration</span><span class="v" id="cRr">--</span></div><div class="kv"><span class="k">Trusted vitals</span><span class="v" id="cTrusted">--</span></div><div class="kv"><span class="k">Packet age</span><span class="v" id="cAge">--</span></div></section>
    <section class="card"><h2>MLX90614</h2><div class="kv"><span class="k">Health</span><span class="v" id="mHealth">--</span></div><div class="kv"><span class="k">Object</span><span class="v" id="mObj">--</span></div><div class="kv"><span class="k">Ambient</span><span class="v" id="mAmb">--</span></div><div class="kv"><span class="k">Target visible</span><span class="v" id="mVisible">--</span></div><div class="kv"><span class="k">Fusion suspended</span><span class="v" id="mSusp">--</span></div><div class="kv"><span class="k">Baseline ready</span><span class="v" id="mBase">--</span></div></section>
    <section class="card"><h2>FSR Pressure</h2><div class="kv"><span class="k">Health</span><span class="v" id="fHealth">--</span></div><div class="kv"><span class="k">Calibrated</span><span class="v" id="fCal">--</span></div><div class="kv"><span class="k">Occupied</span><span class="v" id="fOcc">--</span></div><div class="kv"><span class="k">Whole total</span><span class="v" id="fTotal">--</span></div><div class="kv"><span class="k">Sampling</span><span class="v" id="fHz">--</span></div><div class="fsr" id="fsr"></div></section>
    <section class="card"><h2>MPU6050</h2><div class="kv"><span class="k">Health</span><span class="v" id="pHealth">--</span></div><div class="kv"><span class="k">Accel</span><span class="v" id="pAcc">--</span></div><div class="kv"><span class="k">Gyro</span><span class="v" id="pGyro">--</span></div><div class="kv"><span class="k">Dynamic accel</span><span class="v" id="pDyn">--</span></div><div class="kv"><span class="k">Sampling</span><span class="v" id="pHz">--</span></div><div class="kv"><span class="k">Motion context</span><span class="v" id="motion">--</span></div></section>
    <section class="card"><h2>Camera</h2><div class="kv"><span class="k">Transport</span><span class="v" id="camLink">--</span></div><div class="kv"><span class="k">Ready</span><span class="v" id="camReady">--</span></div><div class="kv"><span class="k">Session</span><span class="v" id="camSess">--</span></div><div class="kv"><span class="k">Baseline</span><span class="v" id="camBase">--</span></div><div class="kv"><span class="k">Posture</span><span class="v" id="camPosture">--</span></div><div class="kv"><span class="k">Packet age</span><span class="v" id="camAge">--</span></div></section>
    <section class="card"><h2>Fusion Evidence</h2><div class="kv"><span class="k">Vitals</span><span class="v" id="eVitals">--</span></div><div class="kv"><span class="k">Pressure</span><span class="v" id="ePressure">--</span></div><div class="kv"><span class="k">Temperature</span><span class="v" id="eTemp">--</span></div><div class="kv"><span class="k">Respiration</span><span class="v" id="eResp">--</span></div><div class="kv"><span class="k">Valid / unavailable</span><span class="v" id="eValid">--</span></div><div class="kv"><span class="k">Anomaly / strong</span><span class="v" id="eAnom">--</span></div></section>
    <section class="card full"><h2>Raw status snapshot</h2><pre class="raw" id="raw">No snapshot yet.</pre></section>
  </div>
  <div class="footer">/uat maintenance monitor • one /api/v1/status request per refresh • no hub-side UAT injection</div>
</div>
<script>
const $=id=>document.getElementById(id),yn=v=>v?'YES':'NO',num=(v,d=1,u='')=>v==null?'--':Number(v).toFixed(d)+u;
let live=false,timer=null,busy=false;
function clsForFusion(v){return v==='SAFE'?'ok':v==='EMERGENCY'?'bad':v==='WARNING'?'warn':''}
function set(id,v,c=''){const e=$(id);e.textContent=v;e.className='v '+c}
function toggleRaw(){const e=$('raw');e.style.display=e.style.display==='block'?'none':'block'}
function requestStatus(){return new Promise((resolve,reject)=>{const x=new XMLHttpRequest();x.open('GET','/api/v1/status?_t='+Date.now(),true);x.timeout=5000;x.onreadystatechange=()=>{if(x.readyState!==4)return;if(x.status>=200&&x.status<300){try{resolve(JSON.parse(x.responseText))}catch(e){reject(new Error('Invalid JSON'))}}else reject(new Error('HTTP '+x.status))};x.onerror=()=>reject(new Error('Network error'));x.ontimeout=()=>reject(new Error('Timeout'));x.send()})}
function renderFsr(f){const g=$('fsr');g.innerHTML='';(f.pressure||[]).forEach((v,i)=>{const e=document.createElement('div');e.className='fsrCell';e.innerHTML='<span class="k">FSR'+(i+1)+'</span><strong>'+num(v,1)+'</strong>';g.appendChild(e)})}
function render(d){if(!d.telemetry_ready)throw new Error('Telemetry not ready');const s=d.system,c=d.sensors.c1001,m=d.sensors.mlx90614,f=d.sensors.fsr,p=d.sensors.mpu6050,cam=d.camera,e=s.evidence,n=d.network;
set('hub','ONLINE','ok');set('fusion',s.fusion_state,clsForFusion(s.fusion_state));set('occupancy',s.occupancy);set('clients',String(n.connected_clients));
set('cLink',c.connected&&!c.stale?'CONNECTED':c.stale?'STALE':'OFFLINE',c.connected&&!c.stale?'ok':c.stale?'warn':'bad');set('cPres',yn(c.present));set('cHr',c.trusted_vitals?num(c.heart_rate_bpm,1,' BPM'):'--');set('cRr',c.trusted_vitals?num(c.respiration_rate_bpm,1,' RPM'):'--');set('cTrusted',yn(c.trusted_vitals));set('cAge',c.packet_age_ms+' ms');
set('mHealth',m.health);set('mObj',num(m.object_temperature_c,2,' °C'));set('mAmb',num(m.sensor_ta_c,2,' °C'));set('mVisible',yn(m.target_visible),m.target_visible?'ok':'warn');set('mSusp',yn(m.fusion_contribution_suspended),m.fusion_contribution_suspended?'warn':'');set('mBase',yn(m.context.baseline_ready));
set('fHealth',f.health,f.health==='OK'?'ok':f.health==='DEGRADED'?'warn':'');set('fCal',yn(f.calibrated));set('fOcc',yn(f.occupied));set('fTotal',num(f.whole_seat_total,1));set('fHz',num(f.sampling_rate_hz,2,' Hz'));renderFsr(f);
set('pHealth',p.health);set('pAcc',num(p.accel_magnitude_g,4,' g'));set('pGyro',num(p.gyro_magnitude_dps,3,' dps'));set('pDyn',num(p.dynamic_acceleration_g,4,' g'));set('pHz',num(p.sampling_rate_hz,2,' Hz'));set('motion',s.motion_context);
set('camLink',cam.transport_connected?'CONNECTED':'OFFLINE',cam.transport_connected?'ok':'bad');set('camReady',cam.camera_ready&&cam.model_ready?'READY':'NOT READY');set('camSess',cam.local_session_active?'ACTIVE':'NONE');set('camBase',cam.baseline_ready?'READY':cam.calibrating?cam.calibration_count+'/'+cam.calibration_target:'WAIT');set('camPosture',cam.posture||'--');set('camAge',cam.packet_age_ms+' ms');
set('eVitals',s.vitals_state);set('ePressure',s.pressure_state);set('eTemp',s.temperature_state);set('eResp',s.respiration_state);set('eValid',e.valid_sensor_count+' / '+e.unavailable_sensor_count);set('eAnom',e.anomaly_evidence_count+' / '+e.strong_anomaly_evidence_count);
$('raw').textContent=JSON.stringify(d,null,2);$('msg').textContent='Last snapshot: '+new Date().toLocaleTimeString()+(live?' • Live Maintenance ON (5 s)':' • manual mode');}
async function refreshNow(){if(busy)return;busy=true;$('msg').textContent='Refreshing…';try{const d=await requestStatus();render(d)}catch(e){set('hub','OFFLINE','bad');$('msg').textContent='Refresh failed: '+e.message}finally{busy=false}}
function schedule(){clearTimeout(timer);if(live)timer=setTimeout(async()=>{await refreshNow();schedule()},5000)}
function toggleLive(){live=!live;const b=$('liveBtn');b.textContent='Live Maintenance: '+(live?'ON':'OFF');b.className='btn secondary '+(live?'active':'');if(live)refreshNow();schedule()}
window.addEventListener('load',()=>refreshNow());
</script>
</body>
</html>
)rawliteral";
