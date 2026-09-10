/*
 * livedata_page.h
 *
 * Self-contained "Live Data" page served at GET /livedata by the ESP web
 * server. Not part of the Angular bundle in LittleFS - it ships inside the
 * firmware so it can be added with a plain "pio run -t upload" (no filesystem
 * upload, no loss of the device's settings.json / states.json).
 *
 * It shows the current value + raw response of every configured state and has
 * an interactive scanner for ad-hoc service/DID ranges (POST /api/obd/scan).
 */
#pragma once

static const char LIVEDATA_HTML[] PROGMEM = R"HTMLPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OBD Live Data</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; font: 14px/1.4 system-ui, sans-serif; background: #12151b; color: #d8dee9; }
  header { padding: 10px 14px; background: #1b2129; border-bottom: 1px solid #2b333f; }
  h1 { font-size: 16px; margin: 0; }
  .tabs { display: flex; gap: 4px; padding: 8px 14px 0; background: #1b2129; }
  .tabs button { background: #232b36; color: #9aa5b1; border: 1px solid #2b333f; border-bottom: none;
    padding: 7px 14px; border-radius: 6px 6px 0 0; cursor: pointer; font: inherit; }
  .tabs button.active { background: #12151b; color: #d8dee9; }
  main { padding: 14px; }
  .row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin-bottom: 10px; }
  input, select { background: #232b36; color: #d8dee9; border: 1px solid #3a4453; border-radius: 5px;
    padding: 6px 8px; font: inherit; }
  input[type=text] { width: 90px; }
  button.go { background: #3b82f6; color: #fff; border: none; border-radius: 5px; padding: 7px 16px;
    cursor: pointer; font: inherit; }
  button.go:disabled { background: #394150; cursor: default; }
  label { color: #9aa5b1; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  th, td { text-align: left; padding: 5px 8px; border-bottom: 1px solid #232b36; vertical-align: top; }
  th { color: #9aa5b1; font-weight: 600; position: sticky; top: 0; background: #12151b; }
  code, .hex { font-family: ui-monospace, monospace; color: #93c5fd; word-break: break-all; }
  .muted { color: #6b7480; }
  .ok { color: #4ade80; } .warn { color: #fbbf24; } .err { color: #f87171; }
  .bar { height: 6px; background: #232b36; border-radius: 3px; overflow: hidden; flex: 1; min-width: 120px; }
  .bar > div { height: 100%; background: #3b82f6; width: 0; transition: width .2s; }
  .scroll { overflow-x: auto; }
  small { color: #6b7480; }
</style>
</head>
<body>
<header><h1>OBD Live Data &amp; Scanner</h1></header>
<div class="tabs">
  <button id="tabLiveBtn" class="active" onclick="showTab('live')">Live</button>
  <button id="tabScanBtn" onclick="showTab('scan')">PID / DID Scan</button>
</div>
<main>

<section id="tabLive">
  <div class="row">
    <input type="search" id="filter" placeholder="Filter name/PID..." oninput="renderLive()" style="width:200px">
    <label><input type="checkbox" id="onlyDiag" onchange="renderLive()"> nur Diagnose</label>
    <label><input type="checkbox" id="autoRefresh" checked> Auto (2s)</label>
    <button class="go" onclick="loadLive()">Jetzt</button>
    <span id="liveInfo" class="muted"></span>
  </div>
  <div class="scroll">
  <table id="liveTable">
    <thead><tr>
      <th>Name</th><th>Wert</th><th>Roh (Datenbytes)</th><th>Service/PID/Header</th><th>Alter</th><th>Status</th>
    </tr></thead>
    <tbody></tbody>
  </table>
  </div>
</section>

<section id="tabScan" hidden>
  <div class="row">
    <label>Service <input type="text" id="sService" value="22"></label>
    <label>Header <input type="text" id="sHeader" value="7E5"></label>
    <label>DID von <input type="text" id="sFrom" value="1E00"></label>
    <label>bis <input type="text" id="sTo" value="1EFF"></label>
    <button class="go" id="scanBtn" onclick="startScan()">Scan starten</button>
  </div>
  <div class="row">
    <div class="bar"><div id="scanBar"></div></div>
    <span id="scanInfo" class="muted"></span>
  </div>
  <small>Werte hex. Service 22 = UDS (2-Byte-DID). Header leer/0 = 7DF (Broadcast, 1-Byte-PID).
    Antworten mit NRC 11/31 (nicht unterst&uuml;tzt) werden nicht gelistet.</small>
  <div class="scroll">
  <table id="scanTable">
    <thead><tr><th>DID</th><th>Antwort (roh)</th><th>Datenbytes</th><th>Deutungen</th><th>NRC</th></tr></thead>
    <tbody></tbody>
  </table>
  </div>
</section>

</main>
<script>
const $ = s => document.querySelector(s);
const hx = n => n.toString(16).toUpperCase().padStart(2,'0');
const STATUS = {0:['ok','OK'], 1:['warn','keine Daten'], 2:['warn','getting'], 3:['err','Timeout'],
                4:['err','Bus-Fehler'], 5:['err','keine Antwort'], 6:['err','Fehler']};

function showTab(t){
  $('#tabLive').hidden = t!=='live'; $('#tabScan').hidden = t!=='scan';
  $('#tabLiveBtn').classList.toggle('active', t==='live');
  $('#tabScanBtn').classList.toggle('active', t==='scan');
}

/* ---- data bytes: strip the "43 41.." / "62 DID.." echo ---- */
function dataBytes(raw){
  if(!raw) return '';
  const b = raw.match(/.{2}/g) || [];
  if(b.length>=1 && (parseInt(b[0],16)&0x40)){         // positive response
    const skip = parseInt(b[0],16)===0x62 ? 3 : 2;
    return b.slice(skip).join(' ');
  }
  return b.join(' ');
}
function interp(raw){
  const b = (dataBytes(raw).match(/[0-9A-Fa-f]{2}/g)||[]).map(x=>parseInt(x,16));
  if(!b.length) return '';
  const A=b[0], B=b[1]??0, u16=(A<<8)|B;
  const s16 = u16>0x7FFF ? u16-0x10000 : u16;
  const out = [`A=${A}`, `A-40=${A-40}`];
  if(b.length>=2){ out.push(`u16=${u16}`, `s16=${s16}`, `u16/100=${(u16/100).toFixed(2)}`, `u16/10=${(u16/10).toFixed(1)}`); }
  if(b.length>=3){ const u24=(A<<16)|(B<<8)|b[2]; out.push(`u24=${u24}`); }
  if(b.length>=4){ const u32=((A<<24)|(B<<16)|(b[2]<<8)|b[3])>>>0; out.push(`u32=${u32}`); }
  return out.join('  ');
}

/* ---------------- Live ---------------- */
let liveData = [];
async function loadLive(){
  try{
    const r = await fetch('/api/obd/live'); liveData = await r.json();
    $('#liveInfo').textContent = liveData.length + ' States  ' + new Date().toLocaleTimeString();
    renderLive();
  }catch(e){ $('#liveInfo').textContent = 'Fehler: ' + e; }
}
function renderLive(){
  const f = $('#filter').value.toLowerCase();
  const od = $('#onlyDiag').checked;
  const tb = $('#liveTable').querySelector('tbody'); tb.innerHTML='';
  for(const s of liveData){
    if(od && !s.diag) continue;
    const pidTxt = s.calc ? 'CALC' : (s.svc? hx(s.svc) : '01') + ' ' + (s.pid>0xFF? s.pid.toString(16).toUpperCase().padStart(4,'0') : hx(s.pid)) + (s.hdr? ' @'+s.hdr.toString(16).toUpperCase() : '');
    if(f && !(s.name.toLowerCase().includes(f) || s.desc.toLowerCase().includes(f) || pidTxt.toLowerCase().includes(f))) continue;
    const st = STATUS[s.status] || ['muted','?'];
    const age = s.age<0 ? '&mdash;' : (s.age<10000? (s.age/1000).toFixed(1)+'s' : Math.round(s.age/1000)+'s');
    const tr = document.createElement('tr');
    tr.innerHTML = `<td><b>${s.name}</b>${s.enabled?'':' <small>(aus)</small>'}<br><small>${s.desc||''}</small></td>
      <td>${s.value ?? ''} <span class="muted">${s.unit||''}</span></td>
      <td><span class="hex">${s.raw||'&mdash;'}</span>${s.raw?`<br><small class="hex">${dataBytes(s.raw)}</small>`:''}</td>
      <td><code>${pidTxt}</code></td>
      <td class="${s.age>60000?'warn':''}">${age}</td>
      <td class="${st[0]}">${st[1]}</td>`;
    tb.appendChild(tr);
  }
}
setInterval(()=>{ if(!$('#tabLive').hidden && $('#autoRefresh').checked) loadLive(); }, 2000);

/* ---------------- Scan ---------------- */
let scanTimer = null;
async function startScan(){
  const service = parseInt($('#sService').value,16) || 0x22;
  const header  = parseInt($('#sHeader').value,16) || 0;
  const from    = parseInt($('#sFrom').value,16) || 0;
  const to      = parseInt($('#sTo').value,16) || from;
  $('#scanBtn').disabled = true;
  $('#scanTable').querySelector('tbody').innerHTML = '';
  try{
    const r = await fetch('/api/obd/scan', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({service, header, from, to})});
    if(!r.ok){ $('#scanInfo').textContent = 'Start fehlgeschlagen: ' + await r.text(); $('#scanBtn').disabled=false; return; }
    pollScan();
  }catch(e){ $('#scanInfo').textContent = 'Fehler: ' + e; $('#scanBtn').disabled=false; }
}
async function pollScan(){
  clearTimeout(scanTimer);
  let j;
  try{ j = await (await fetch('/api/obd/scan')).json(); }
  catch(e){ scanTimer = setTimeout(pollScan, 1200); return; }
  const pct = j.total ? Math.round(100*j.done/j.total) : 0;
  $('#scanBar').style.width = pct + '%';
  $('#scanInfo').textContent = `${j.done}/${j.total} DIDs  (${j.results.length} Treffer)` + (j.running?' ...':' - fertig');
  const tb = $('#scanTable').querySelector('tbody'); tb.innerHTML='';
  for(const res of j.results){
    const did = res.pid>0xFF ? res.pid.toString(16).toUpperCase().padStart(4,'0') : hx(res.pid);
    const tr = document.createElement('tr');
    tr.innerHTML = `<td><code>${did}</code></td>
      <td><span class="hex">${res.raw||''}</span></td>
      <td><span class="hex">${dataBytes(res.raw)}</span></td>
      <td><small>${res.raw ? interp(res.raw) : ''}</small></td>
      <td class="${res.nrc?'muted':''}">${res.nrc? '0x'+hx(res.nrc) : ''}</td>`;
    tb.appendChild(tr);
  }
  if(j.running){ scanTimer = setTimeout(pollScan, 800); }
  else { $('#scanBtn').disabled = false; }
}

loadLive();
</script>
</body>
</html>
)HTMLPAGE";
