#pragma once

#include <Arduino.h>

// ============================================================
// SAFESEAT UAT EVALUATOR MONITOR
//
// Read-only browser dashboard served by the Main Hub at /uat.
// It is intentionally separate from the participant-facing mobile app.
// Evaluators can connect a second phone/laptop to the SafeSeat AP and
// observe the same telemetry used by Fusion without exposing raw values
// on the participant dashboard.
// ============================================================

static const char SAFESEAT_UAT_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>SafeSeat UAT Evaluator Monitor</title>
<style>
:root{color-scheme:dark;--bg:#0b1017;--panel:#151c26;--panel2:#101720;--line:#283445;--text:#f5f7fa;--muted:#9eabbc;--ok:#4bd37b;--warn:#f5c14f;--bad:#ff6b6b;--accent:#55a7ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}header{position:sticky;top:0;z-index:5;background:rgba(11,16,23,.94);backdrop-filter:blur(8px);border-bottom:1px solid var(--line);padding:14px 18px}.title{display:flex;align-items:center;gap:10px;font-size:20px;font-weight:800}.badge{font-size:12px;padding:4px 8px;border-radius:999px;background:#27364b;color:#cfe6ff}.sub{color:var(--muted);font-size:12px;margin-top:4px}.wrap{max-width:1200px;margin:auto;padding:16px}.top{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-bottom:12px}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.card,.mini{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}.card h2{margin:0 0 12px;font-size:16px}.mini .v{font-size:22px;font-weight:800;margin-top:4px}.mini .k,.kv .k{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.06em}.kv{display:grid;grid-template-columns:1.4fr 1fr;gap:8px;padding:6px 0;border-bottom:1px solid #202a38}.kv:last-child{border-bottom:0}.kv .v{text-align:right;font-variant-numeric:tabular-nums}.status{display:inline-flex;align-items:center;gap:7px}.dot{width:9px;height:9px;border-radius:50%;background:var(--muted)}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}.dot.ok{background:var(--ok)}.dot.warn{background:var(--warn)}.dot.bad{background:var(--bad)}.fsr{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}.fsrCell{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:9px}.fsrCell strong{display:block;font-size:18px;font-variant-numeric:tabular-nums}.bar{height:5px;background:#263244;border-radius:99px;overflow:hidden;margin-top:6px}.bar i{display:block;height:100%;background:var(--accent);width:0}.model{margin-top:10px;padding:9px;background:var(--panel2);border-radius:9px;font-size:12px;color:var(--muted)}.footer{padding:18px 0;color:var(--muted);font-size:11px;text-align:center}.raw{white-space:pre-wrap;word-break:break-word;background:#080c12;border:1px solid var(--line);border-radius:10px;padding:10px;max-height:300px;overflow:auto;font:11px ui-monospace,SFMono-Regular,Consolas,monospace}button{background:#1e6fb9;color:white;border:0;border-radius:9px;padding:9px 12px;font-weight:700;cursor:pointer}.actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}input,select{width:100%;background:var(--panel2);color:var(--text);border:1px solid var(--line);border-radius:9px;padding:9px}.metaGrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.rec{font-size:12px;color:var(--muted);margin-top:8px}@media(max-width:620px){.metaGrid{grid-template-columns:1fr}}@media(max-width:820px){.top{grid-template-columns:repeat(2,1fr)}.grid{grid-template-columns:1fr}}@media(max-width:460px){.top{grid-template-columns:1fr 1fr}.mini .v{font-size:18px}}
</style>
</head>
<body>
<header>
  <div class="title">SafeSeat UAT Evaluator Monitor <span class="badge">DRY-RUN MODE</span></div>
  <div class="sub">Evaluator-only technical view. Raw telemetry is read-only; camera verification is triggered only by the production Fusion path.</div>
</header>
<div class="wrap">
  <section class="card" style="margin-bottom:12px">
    <h2>UAT Run Metadata <span class="badge">EVALUATOR LOCAL ONLY</span></h2>
    <div class="metaGrid">
      <label><span class="k">Physical prototype position</span><select id="meta-seat">
        <option value="Driver">Driver</option><option value="Front Passenger">Front Passenger</option>
        <option value="Rear Left">Rear Left</option><option value="Rear Center">Rear Center</option><option value="Rear Right">Rear Right</option>
      </select></label>
      <label><span class="k">UAT participant / run code</span><input id="meta-run" maxlength="40" placeholder="e.g. UAT-P07"></label>
    </div>
    <div class="actions"><button id="rec-btn" onclick="toggleRecording()">Start Recording</button><button onclick="downloadCsv()">Download CSV</button><button onclick="clearRecording()">Clear Recording</button></div>
    <div class="rec" id="rec-status">Not recording • 0 samples. Metadata stays in this browser and does not affect Fusion.</div>
  </section>

  <div class="top">
    <div class="mini"><div class="k">Hub/API</div><div class="v" id="hub">CONNECTING</div></div>
    <div class="mini"><div class="k">Fusion</div><div class="v" id="fusion">--</div></div>
    <div class="mini"><div class="k">Occupancy</div><div class="v" id="occupancy">--</div></div>
    <div class="mini"><div class="k">Last update</div><div class="v" id="last">--</div></div>
  </div>

  <div class="grid">
    <section class="card">
      <h2>M1 • Headrest — C1001 + MLX90614</h2>
      <div class="kv"><span class="k">C1001 link</span><span class="v" id="c-link">--</span></div>
      <div class="kv"><span class="k">Presence</span><span class="v" id="c-pres">--</span></div>
      <div class="kv"><span class="k">Heart rate</span><span class="v" id="c-hr">--</span></div>
      <div class="kv"><span class="k">Respiration rate</span><span class="v" id="c-rr">--</span></div>
      <div class="kv"><span class="k">Trusted vitals</span><span class="v" id="c-trusted">--</span></div>
      <div class="kv"><span class="k">Motion / MoveRange</span><span class="v" id="c-motion">--</span></div>
      <div class="kv"><span class="k">Motion artifact</span><span class="v" id="c-artifact">--</span></div>
      <div class="model" id="c-model">Model: --</div>
      <hr style="border:0;border-top:1px solid var(--line);margin:12px 0">
      <div class="kv"><span class="k">MLX object</span><span class="v" id="m-obj">--</span></div>
      <div class="kv"><span class="k">MLX ambient</span><span class="v" id="m-amb">--</span></div>
      <div class="kv"><span class="k">Object − ambient</span><span class="v" id="m-delta">--</span></div>
      <div class="kv"><span class="k">Target qualified</span><span class="v" id="m-target">--</span></div>
      <div class="kv"><span class="k">Baseline / deviation</span><span class="v" id="m-base">--</span></div>
      <div class="model" id="m-model">Model: --</div>
    </section>

    <section class="card">
      <h2>M2 + M3 • FSR Pressure — Backrest + Cushion</h2>
      <div class="kv"><span class="k">FSR health</span><span class="v" id="f-health">--</span></div>
      <div class="kv"><span class="k">Occupied</span><span class="v" id="f-occ">--</span></div>
      <div class="kv"><span class="k">Backrest total</span><span class="v" id="f-back">--</span></div>
      <div class="kv"><span class="k">Cushion total</span><span class="v" id="f-cush">--</span></div>
      <div class="kv"><span class="k">Whole-seat total</span><span class="v" id="f-total">--</span></div>
      <div class="kv"><span class="k">Sampling</span><span class="v" id="f-hz">--</span></div>
      <div class="fsr" id="fsr-grid"></div>
      <div class="model" id="f-model">Model: --</div>
    </section>

    <section class="card">
      <h2>M4 • Seat Frame — MPU6050</h2>
      <div class="kv"><span class="k">Health</span><span class="v" id="p-health">--</span></div>
      <div class="kv"><span class="k">Accel magnitude</span><span class="v" id="p-acc">--</span></div>
      <div class="kv"><span class="k">Gyro magnitude</span><span class="v" id="p-gyro">--</span></div>
      <div class="kv"><span class="k">Dynamic acceleration</span><span class="v" id="p-dyn">--</span></div>
      <div class="kv"><span class="k">Sampling</span><span class="v" id="p-hz">--</span></div>
      <div class="kv"><span class="k">Fusion motion context</span><span class="v" id="motionctx">--</span></div>
      <div class="model" id="p-model">Model: --</div>
    </section>

    <section class="card">
      <h2>CAM • Trigger-only Posture Verification</h2>
      <div class="kv"><span class="k">Transport / ready</span><span class="v" id="cam-ready">--</span></div>
      <div class="kv"><span class="k">Passenger session (Hub / Camera)</span><span class="v" id="cam-session">--</span></div>
      <div class="kv"><span class="k">Upright baseline</span><span class="v" id="cam-baseline">--</span></div>
      <div class="kv"><span class="k">Verification requested</span><span class="v" id="cam-req">--</span></div>
      <div class="kv"><span class="k">Busy</span><span class="v" id="cam-busy">--</span></div>
      <div class="kv"><span class="k">Posture</span><span class="v" id="cam-posture">--</span></div>
      <div class="kv"><span class="k">Confidence</span><span class="v" id="cam-conf">--</span></div>
      <div class="kv"><span class="k">Result valid</span><span class="v" id="cam-valid">--</span></div>
      <div class="kv"><span class="k">Packet age</span><span class="v" id="cam-age">--</span></div>
      <div class="kv"><span class="k">Alert requested</span><span class="v" id="alert">--</span></div>
    </section>

    <section class="card">
      <h2>Fusion Evidence</h2>
      <div class="kv"><span class="k">Vitals</span><span class="v" id="e-vitals">--</span></div>
      <div class="kv"><span class="k">Pressure</span><span class="v" id="e-pressure">--</span></div>
      <div class="kv"><span class="k">Temperature</span><span class="v" id="e-temp">--</span></div>
      <div class="kv"><span class="k">Respiration</span><span class="v" id="e-resp">--</span></div>
      <div class="kv"><span class="k">Valid / unavailable sensors</span><span class="v" id="e-valid">--</span></div>
      <div class="kv"><span class="k">Anomaly / strong anomaly</span><span class="v" id="e-anom">--</span></div>
      <div class="kv"><span class="k">Normal / supporting context</span><span class="v" id="e-normal">--</span></div>
      <div class="kv"><span class="k">Multi-sensor agreement</span><span class="v" id="e-agree">--</span></div>
      <div class="kv"><span class="k">Motion artifact possible</span><span class="v" id="e-artifact">--</span></div>
    </section>

    <section class="card">
      <h2>Network / Raw Snapshot</h2>
      <div class="kv"><span class="k">SafeSeat AP</span><span class="v" id="n-ap">--</span></div>
      <div class="kv"><span class="k">Channel</span><span class="v" id="n-ch">--</span></div>
      <div class="kv"><span class="k">Connected clients</span><span class="v" id="n-clients">--</span></div>
      <div class="kv"><span class="k">Hub uptime</span><span class="v" id="uptime">--</span></div>
      <div class="actions"><button onclick="toggleRaw()">Show / hide JSON</button></div>
      <pre class="raw" id="raw" style="display:none">Waiting for telemetry…</pre>
    </section>
  </div>
  <div class="footer">SafeSeat UAT evaluator view • Read-only telemetry • No medical diagnosis is produced by this page.</div>
</div>
<script>
const $=id=>document.getElementById(id);const yn=v=>v?'YES':'NO';
let recording=false,records=[];
const seatMeta=$('meta-seat'),runMeta=$('meta-run'),recStatus=$('rec-status'),recBtn=$('rec-btn');
seatMeta.value=localStorage.getItem('safeseat_uat_seat')||'Driver';runMeta.value=localStorage.getItem('safeseat_uat_run')||'';
seatMeta.addEventListener('change',()=>localStorage.setItem('safeseat_uat_seat',seatMeta.value));runMeta.addEventListener('input',()=>localStorage.setItem('safeseat_uat_run',runMeta.value));
function updateRec(){recBtn.textContent=recording?'Stop Recording':'Start Recording';recStatus.textContent=`${recording?'RECORDING':'Not recording'} • ${records.length} samples • ${seatMeta.value}${runMeta.value?' • '+runMeta.value:''}. Metadata stays in this browser and does not affect Fusion.`}
function toggleRecording(){recording=!recording;updateRec()}
function clearRecording(){recording=false;records=[];updateRec()}
function csvEscape(v){const t=String(v??'');return /[",\n]/.test(t)?'"'+t.replace(/"/g,'""')+'"':t}
function rowFrom(d){const s=d.system,c=d.sensors.c1001,m=d.sensors.mlx90614,f=d.sensors.fsr,p=d.sensors.mpu6050,cam=d.camera,e=s.evidence;const r={recorded_at:new Date().toISOString(),uat_run:runMeta.value||'',prototype_position:seatMeta.value,fusion_state:s.fusion_state,occupancy:s.occupancy,alert_requested:s.alert_requested,c1001_present:c.present,hr_bpm:c.heart_rate_bpm,rr_bpm:c.respiration_rate_bpm,trusted_vitals:c.trusted_vitals,motion:c.motion,move_range:c.move_range,motion_artifact:c.motion_artifact_active,mlx_object_c:m.object_temperature_c,mlx_ambient_c:m.sensor_ta_c,mlx_delta_c:m.object_minus_ta_c,fsr_backrest_total:f.backrest_total,fsr_cushion_total:f.cushion_total,fsr_whole_total:f.whole_seat_total,fsr_occupied:f.occupied,mpu_accel_g:p.accel_magnitude_g,mpu_gyro_dps:p.gyro_magnitude_dps,mpu_dynamic_g:p.dynamic_acceleration_g,motion_context:s.motion_context,camera_requested:cam.verification_requested,camera_posture:cam.posture,camera_confidence:cam.confidence,camera_result_valid:cam.result_valid,valid_sensor_count:e.valid_sensor_count,anomaly_evidence_count:e.anomaly_evidence_count,strong_anomaly_evidence_count:e.strong_anomaly_evidence_count};(f.pressure||[]).forEach((v,i)=>r['fsr'+(i+1)]=v);(f.pressure_share||[]).forEach((v,i)=>r['fsr'+(i+1)+'_share']=v);return r}
function downloadCsv(){if(!records.length){alert('No evaluator samples recorded yet.');return}const headers=Object.keys(records[0]);const csv=[headers.join(','),...records.map(r=>headers.map(h=>csvEscape(r[h])).join(','))].join('\n');const blob=new Blob([csv],{type:'text/csv'}),a=document.createElement('a'),safe=(runMeta.value||'uat').replace(/[^a-z0-9_-]+/gi,'_');a.href=URL.createObjectURL(blob);a.download=`SafeSeat_${safe}_${seatMeta.value.replace(/\s+/g,'_')}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
updateRec();const num=(v,d=1,u='')=>v==null?'--':Number(v).toFixed(d)+u;const model=m=>{if(!m)return'Model: --';const cls=m.both_models_anomaly?'BOTH ANOMALY':m.either_model_anomaly?'EITHER ANOMALY':'NORMAL';return `Model: ${cls} • IF ${num(m.isolation_forest_score,4)} • OCSVM ${num(m.one_class_svm_score,4)} • conf ${num(m.confidence,2)}`};
function setStatus(el,text,state){el.textContent=text;el.className='v '+(state||'')}
function toggleRaw(){const r=$('raw');r.style.display=r.style.display==='none'?'block':'none'}
function renderFsr(f){const g=$('fsr-grid');g.innerHTML='';const values=f.pressure||[];const shares=f.pressure_share||[];values.forEach((v,i)=>{const d=document.createElement('div');d.className='fsrCell';const s=Math.max(0,Math.min(1,Number(shares[i]||0)));d.innerHTML=`<span class="k">FSR${i+1}${i<6?' Backrest':' Cushion'}</span><strong>${v==null?'--':Number(v).toFixed(1)}</strong><span class="k">share ${(s*100).toFixed(1)}%</span><div class="bar"><i style="width:${(s*100).toFixed(1)}%"></i></div>`;g.appendChild(d)})}
function render(d){const s=d.system,c=d.sensors.c1001,m=d.sensors.mlx90614,f=d.sensors.fsr,p=d.sensors.mpu6050,cam=d.camera,e=s.evidence,n=d.network;
 setStatus($('hub'),'ONLINE','ok');setStatus($('fusion'),s.fusion_state,s.fusion_state==='SAFE'?'ok':s.fusion_state==='EMERGENCY'?'bad':'warn');$('occupancy').textContent=s.occupancy;$('last').textContent=new Date().toLocaleTimeString();
 $('c-link').textContent=(c.connected&&!c.stale)?'CONNECTED':c.stale?'STALE':'OFFLINE';$('c-pres').textContent=yn(c.present);$('c-hr').textContent=c.trusted_vitals?num(c.heart_rate_bpm,1,' BPM'):'--';$('c-rr').textContent=c.trusted_vitals?num(c.respiration_rate_bpm,1,' RPM'):'--';$('c-trusted').textContent=yn(c.trusted_vitals);$('c-motion').textContent=`${c.motion} / ${c.move_range}`;$('c-artifact').textContent=yn(c.motion_artifact_active);$('c-model').textContent=model(c.model);
 $('m-obj').textContent=num(m.object_temperature_c,2,' °C');$('m-amb').textContent=num(m.sensor_ta_c,2,' °C');$('m-delta').textContent=num(m.object_minus_ta_c,2,' °C');$('m-target').textContent=yn(m.context.thermal_target_qualified);$('m-base').textContent=`${num(m.context.baseline_object_c,2,' °C')} / ${num(m.context.deviation_from_baseline_c,2,' °C')}`;$('m-model').textContent=model(m.native_mlx_model);
 $('f-health').textContent=`${f.health} • ${f.calibrated?'CALIBRATED':'NOT CALIBRATED'}`;$('f-occ').textContent=yn(f.occupied);$('f-back').textContent=num(f.backrest_total,1);$('f-cush').textContent=num(f.cushion_total,1);$('f-total').textContent=num(f.whole_seat_total,1);$('f-hz').textContent=num(f.sampling_rate_hz,1,' Hz');$('f-model').textContent=model(f.model);renderFsr(f);
 $('p-health').textContent=p.health;$('p-acc').textContent=num(p.accel_magnitude_g,4,' g');$('p-gyro').textContent=num(p.gyro_magnitude_dps,3,' dps');$('p-dyn').textContent=num(p.dynamic_acceleration_g,4,' g');$('p-hz').textContent=num(p.sampling_rate_hz,1,' Hz');$('motionctx').textContent=s.motion_context;$('p-model').textContent=model(p.road_motion_model);
 $('cam-ready').textContent=`${cam.transport_connected?'LINK':'NO LINK'} / ${cam.camera_ready&&cam.model_ready?'READY':'NOT READY'}`;$('cam-session').textContent=`${cam.local_session_active?'ACTIVE':'NONE'} / ${cam.session_active?'ACTIVE':'NONE'}`;$('cam-baseline').textContent=cam.baseline_ready?'READY':cam.calibrating?`${cam.calibration_count}/${cam.calibration_target} CALIBRATING`:'WAITING';$('cam-req').textContent=yn(cam.verification_requested);$('cam-busy').textContent=yn(cam.busy);$('cam-posture').textContent=cam.posture||'--';$('cam-conf').textContent=num(cam.confidence,3);$('cam-valid').textContent=yn(cam.result_valid);$('cam-age').textContent=cam.packet_age_ms+' ms';$('alert').textContent=yn(s.alert_requested);
 $('e-vitals').textContent=s.vitals_state;$('e-pressure').textContent=s.pressure_state;$('e-temp').textContent=s.temperature_state;$('e-resp').textContent=s.respiration_state;$('e-valid').textContent=`${e.valid_sensor_count} / ${e.unavailable_sensor_count}`;$('e-anom').textContent=`${e.anomaly_evidence_count} / ${e.strong_anomaly_evidence_count}`;$('e-normal').textContent=`${e.normal_evidence_count} / ${e.supporting_context_count}`;$('e-agree').textContent=yn(e.multi_sensor_agreement);$('e-artifact').textContent=yn(e.motion_artifact_possible);
 $('n-ap').textContent=`${n.ssid} • ${n.ip}`;$('n-ch').textContent=n.channel;$('n-clients').textContent=n.connected_clients;$('uptime').textContent=Math.round(d.uptime_ms/1000)+' s';$('raw').textContent=JSON.stringify(d,null,2);
}
const HUB_BASE=(location.hostname==='192.168.4.1'&&location.protocol==='http:')?location.origin:'http://192.168.4.1';
let pollBusy=false,pollTimer=null;
function requestJson(path,timeoutMs=5000){return new Promise((resolve,reject)=>{const x=new XMLHttpRequest();const sep=path.includes('?')?'&':'?';x.open('GET',HUB_BASE+path+sep+'_t='+Date.now(),true);x.timeout=timeoutMs;x.setRequestHeader('Accept','application/json');x.onreadystatechange=()=>{if(x.readyState!==4)return;if(x.status>=200&&x.status<300){try{resolve(JSON.parse(x.responseText))}catch(e){reject(new Error('Invalid JSON from '+path+' ('+x.responseText.length+' bytes)'))}}else{reject(new Error('HTTP '+x.status+' from '+path))}};x.onerror=()=>reject(new Error('Network error requesting '+path));x.ontimeout=()=>reject(new Error('Timeout requesting '+path));x.send()})}
async function readSnapshot(){const fusion=await requestJson('/api/v1/fusion',3000);if(!fusion.telemetry_ready)return fusion;const sensors=await requestJson('/api/v1/sensors',4000);const camera=await requestJson('/api/v1/camera',2500);const network=await requestJson('/api/v1/network',2500);return{schema_version:fusion.schema_version,telemetry_ready:true,timestamp_ms:fusion.timestamp_ms,uptime_ms:fusion.uptime_ms,system:fusion.system,sensors,camera,network}}
async function tick(){if(pollBusy)return;pollBusy=true;try{const d=await readSnapshot();if(!d.telemetry_ready){setStatus($('hub'),'DEGRADED','warn');$('last').textContent='Hub reachable; telemetry not ready';return}render(d);if(recording){records.push(rowFrom(d));updateRec()}}catch(statusErr){try{const h=await requestJson('/health',2500);if(h&&h.ok){setStatus($('hub'),'DEGRADED','warn');$('last').textContent='Hub reachable; split telemetry poll failed: '+String(statusErr.message||statusErr)}else{throw new Error('Health check invalid')}}catch(healthErr){setStatus($('hub'),'OFFLINE','bad');$('last').textContent=String(healthErr.message||healthErr)}}finally{pollBusy=false;clearTimeout(pollTimer);pollTimer=setTimeout(tick,1500)}}
window.addEventListener('load',()=>{pollTimer=setTimeout(tick,750)});
</script>
</body>
</html>
)rawliteral";
