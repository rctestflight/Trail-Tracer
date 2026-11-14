'use strict';
const ESPRESSIF = 0x303A, ROM_PID = 0x0002, APP_PID = 0x80C2;
const PAGE_VERSION = '1.17.0';
let pageVer = `Page v${PAGE_VERSION} (local copy)`;
const ESPTOOL = 'https://unpkg.com/esptool-js@0.4.7/bundle.js';
const ESPTOOL_SRI = 'sha384-esoYAtcIwSlHrIf+6+uZc/SPs/S8v44K26De13juLayamqHofUvCPscyHBXPxVIc';
async function loadEsptool() {
  const text = await (await fetch(ESPTOOL, { integrity: ESPTOOL_SRI, cache: 'no-store' })).text();
  return import(URL.createObjectURL(new Blob([text], { type: 'text/javascript' })));
}
const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const sleep = ms => new Promise(r => setTimeout(r, ms));
const enc = new TextEncoder();
const boards = new Map();
const openPorts = new Set();
const streamLocks = new WeakMap();
for (const [C, m] of [[globalThis.ReadableStream, 'getReader'], [globalThis.WritableStream, 'getWriter']]) {
  if (!C || C.prototype[m].guarded) continue;
  const orig = C.prototype[m];
  C.prototype[m] = function (...a) { const l = orig.apply(this, a); streamLocks.set(this, l); return l; };
  C.prototype[m].guarded = true;
}
function guardPort(port) {
  if (!port || port.guarded) return port;
  const open = port.open.bind(port), close = port.close.bind(port);
  port.open = async o => { await open(o); openPorts.add(port); };
  port.close = async () => { try { await close(); } finally { if (!port.readable && !port.writable) openPorts.delete(port); } };
  port.guarded = true;
  return port;
}
async function forceClose(port) {
  if (!port) return;
  const r = port.readable && streamLocks.get(port.readable), w = port.writable && streamLocks.get(port.writable);
  if (r) { await Promise.race([r.cancel().catch(() => {}), sleep(500)]); try { r.releaseLock(); } catch { /* released */ } }
  if (w) { await Promise.race([w.abort().catch(() => {}), sleep(500)]); try { w.releaseLock(); } catch { /* released */ } }
  if (port.readable || port.writable) await Promise.race([port.close().catch(() => {}), sleep(1000)]);
  openPorts.delete(port);
}
async function releaseAll(keep = null) {
  for (const port of [...openPorts]) if (port !== keep) await forceClose(port);
}
let builds = [];
let job = null;
let bridgeNext = false;
let expectRover = false;

const perm = (() => { try { return JSON.parse(localStorage.getItem('tt.perm') || '{}'); } catch { return {}; } })();
function notePerm(mode) { if (perm[mode]) return; perm[mode] = true; try { localStorage.setItem('tt.perm', JSON.stringify(perm)); } catch { /* no storage */ } }
const firstVisit = () => !perm.app && !perm.rom;
function newBoard(port, state) {
  guardPort(port);
  const b = { port, info: port.getInfo(), state, lines: [], buf: '', waiters: [], binMode: false, bin: new Uint8Array(0), binWait: null, t0: Date.now() };
  b.mode = b.info.usbProductId === ROM_PID ? 'rom' : 'app';
  notePerm(b.mode);
  boards.set(port, b);
  return b;
}
async function open(port) {
  const old = boards.get(port);
  if (old && (old.state === 'open' || old.state === 'opening' || !(job && !job.done))) return old;
  if (old) boards.delete(port);
  const b = newBoard(port, 'opening');
  if (job && !job.done) selPort = port;
  if (b.mode === 'rom') { Object.assign(b, { state: 'open', firmware: ROM_FW, connected: true }); render(); boardOpened(b); return b; }
  render();
  try { await port.open({ baudRate: 115200 }); }
  catch (ex) { say(`port open ${usbId(port)}: ${ex.message}`); b.state = `does not open: ${ex.message}`; render(); return b; }
  say(`port open: ${usbId(port)}`);
  b.state = 'open'; b.connected = true; render();
  if (bridgeNext) { b.bridge = true; b.binMode = true; b.firmware = 'STM32 bridge (IAP)'; b.readDone = readLoop(b); boardOpened(b); return b; }
  b.readDone = readLoop(b);
  detect(b);
  boardOpened(b);
  return b;
}
async function readLoop(b) {
  b.reader = b.port.readable.getReader();
  const dec = new TextDecoder();
  try {
    for (;;) {
      const { value, done } = await b.reader.read();
      if (done) break;
      if (b.binMode) { const n = new Uint8Array(b.bin.length + value.length); n.set(b.bin); n.set(value, b.bin.length); b.bin = n; if (b.binWait) b.binWait(); continue; }
      b.buf += dec.decode(value, { stream: true });
      const parts = b.buf.split(/\r?\n/); b.buf = parts.pop();
      for (const line of parts) if (line.trim()) gotLine(b, line);
      render();
    }
  } catch (ex) { b.state = `closed: ${ex.message}`; }
  finally { try { b.reader.releaseLock(); } catch { /* gone */ } }
  if (!b.closing) { b.closing = true; await b.port.close().catch(() => {}); boards.delete(b.port); }
  render();
}
function binLine(b) {
  b.binLines = (b.binLines || 0) + 1;
  const t = `(${b.binLines} lines of bytes that are not text, not shown)`;
  if (b.lines.length && /^\(\d+ lines of bytes that are not text/.test(b.lines[b.lines.length - 1])) b.lines[b.lines.length - 1] = t;
  else b.lines.push(t);
}
function gotLine(b, line) {
  if (/[\uFFFD\x00-\x08\x0E-\x1F]/.test(line)) binLine(b); else b.lines.push(line);
  if (b.lines.length > 600) b.lines.shift();
  learn(b, line);
  for (const w of [...b.waiters]) if (w.test(line)) { b.waiters.splice(b.waiters.indexOf(w), 1); w.resolve(line); }
}
async function send(b, text) {
  if (!b.port.writable) throw new Error('The board left USB.');
  const w = b.port.writable.getWriter();
  const t = setTimeout(() => w.abort().catch(() => {}), 3000);
  try { await w.write(typeof text === 'string' ? enc.encode(text + '\n') : text); } finally { clearTimeout(t); try { w.releaseLock(); } catch { /* aborted */ } }
}
function waitLine(b, re, ms) {
  return new Promise(resolve => {
    const w = { test: l => re.test(l), resolve };
    b.waiters.push(w);
    setTimeout(() => { const i = b.waiters.indexOf(w); if (i >= 0) { b.waiters.splice(i, 1); resolve(null); } }, ms);
  });
}
async function ask(b, cmd, re, ms = 3000) { const p = waitLine(b, re, ms); await send(b, cmd); return p; }
async function closeBoard(b) {
  b.closing = true;
  if (b.rom) { const r = b.rom; b.rom = null; await romLinkClose(b, r); }
  try { if (b.reader) await b.reader.cancel(); } catch { /* gone */ }
  await b.readDone;
  try { await b.port.close(); } catch (ex) { if (b.port.readable) say(`port close: ${ex.message}`); }
  boards.delete(b.port); render();
}
const ROM_FW = 'none running (ROM download mode)';
let selPort = null;
const listed = () => [...boards.values()].sort((a, c) => a.t0 - c.t0);
function selected() {
  if (job && !job.done) return boards.get(selPort) || null;
  return boards.get(selPort) || null;
}
function choose(port) { if (job && !job.done) return; selPort = port; render(); }
function known(port) {
  if (job && !job.done) return open(port);
  if (boards.has(port)) return boards.get(port);
  const b = newBoard(port, 'closed');
  if (b.mode === 'rom') b.firmware = ROM_FW;
  render();
  return b;
}
async function ensureOpen(b) {
  if (b.mode === 'rom' || b.state === 'open') return b;
  Object.assign(b, { closing: false, buf: '', state: 'opening' }); render();
  try { await b.port.open({ baudRate: 115200 }); }
  catch (ex) { b.state = `does not open: ${ex.message}`; render(); throw plain(`The Rx does not open. ${BUTTON_STEP} Then press Connect.`); }
  b.state = 'open'; b.readDone = readLoop(b); render();
  return b;
}
async function releaseBoard(b) {
  if (b.rom) { const r = b.rom; b.rom = null; await romLinkClose(b, r); if (b.port.readable) await b.port.close().catch(() => {}); }
  if (b.mode !== 'rom' && b.state === 'open') {
    b.closing = true;
    try { if (b.reader) await b.reader.cancel(); } catch { /* gone */ }
    await b.readDone;
    try { await b.port.close(); } catch { /* closed */ }
  }
  Object.assign(b, { reader: null, state: 'closed' }); render();
}
async function connectBoard(b) {
  if (!b || (job && !job.done)) return;
  job = null;
  Object.assign(b, { connected: true, error: null, firmware: b.mode === 'rom' ? ROM_FW : null, version: null, role: null, stm: null, stmLink: null,
    table: null, tableCrc: null, hb: false, redetect: 0, flash: null });
  render();
  try {
    if (b.mode === 'rom') await romRead(b);
    else { await ensureOpen(b); await detect(b); await releaseBoard(b); }
  } catch (ex) { Object.assign(b, { connected: false, error: ex.plain ? ex.message : `The Rx does not answer. ${BUTTON_STEP} Then press Connect.` }); render(); }
}
async function disconnectBoard(b) {
  if (!b || (job && !job.done)) return;
  job = null; b.connected = false;
  await releaseBoard(b);
}

const openWaiters = [];
function boardOpened(b) { for (const w of [...openWaiters]) if (w.test(b)) { openWaiters.splice(openWaiters.indexOf(w), 1); w.resolve(b); } }
const CONNECT_STEP = 'Hold the button while you plug in USB';
const BUTTON_STEP = CONNECT_STEP;
const REPLUG_STEP = 'Unplug, wait 3 s, plug in again.';
const ADD_HINT = 'Press Find Rx Boards and select the Rx.';
const PERM_HINT = 'Press Find Rx Boards and select the Rx again.';
let lastConnect = 0;
let disconnects = 0;
const plain = msg => Object.assign(new Error(msg), { plain: true });
function waitGone(ms) {
  const d0 = disconnects, t0 = Date.now();
  return new Promise(res => { const iv = setInterval(() => { if (disconnects !== d0 || Date.now() - t0 > ms) { clearInterval(iv); res(disconnects !== d0); } }, 100); });
}
async function personPowerCycle(test, text = BUTTON_STEP) {
  const want = x => test(x) || x.mode === 'rom';
  instruct(text);
  // factory mode: a wait on a person has no time limit; "Start over" is the only exit
  if ([...boards.values()].some(x => x.state === 'open') && !await waitGone(FACTORY ? Infinity : 300000)) throw plain(text);
  let b = await waitBoard(want, 10000);
  while (!b && Date.now() - lastConnect < 6000) b = await waitBoard(want, 3000);
  if (!b) { hintRom = false; instruct(`${text.replace(/\.?$/, '.')} ${ADD_HINT}`); b = await waitBoard(want, FACTORY ? Infinity : 170000); }
  if (!b) throw plain(text);
  instruct('');
  if (b.mode === 'rom' && !test(b)) {
    say('the board is in ROM download mode: the watchdog reset starts its firmware');
    await withLoader(b, async loader => {
      const held = async () => ((await loader.readReg(0x3F40403C).catch(() => 1)) & 1) === 0;
      for (let i = 0; i < 34 && await held(); i++) { if (i === 33) instruct('Let go of the button.'); await sleep(300); }
      for (let i = 0; i < 200 && await held(); i++) await sleep(300);
      instruct('');
      await loader.writeReg(S2.OPTION1, 0, S2.FORCE_DL).catch(() => {});
      await watchdogReset(loader);
    });
    b = await waitBoardHint(test, 45000, 'the board');
    if (!b) throw plain(BUTTON_STEP);
  }
  return b;
}
async function waitBoardHint(test, ms, choose) {
  let b = await waitBoard(test, 8000), waited = 8000;
  while (!b && Date.now() - lastConnect < 8000 && waited < ms) { b = await waitBoard(test, 4000); waited += 4000; }
  if (b) return b;
  hintRom = /ESP32-S2/.test(choose || '');
  instruct(perm.app && perm.rom ? ADD_HINT : PERM_HINT);
  return waitBoard(test, FACTORY ? Infinity : Math.max(1000, ms - waited));
}
function waitBoard(test, ms, label) {
  return new Promise(resolve => {
    for (const b of boards.values()) if (b.state === 'open' && test(b)) return resolve(b);
    const w = { test, resolve }; openWaiters.push(w);
    if (label) say(label);
    if (ms !== Infinity) setTimeout(() => { const i = openWaiters.indexOf(w); if (i >= 0) { openWaiters.splice(i, 1); resolve(null); } }, ms);
  });
}
let hintRom = false;                                   // the device a pick asks for: update mode or the firmware

function learn(b, line) {
  let m;
  if ((m = line.match(/^BOOT rover-firmware (\S+)/))) {
    Object.assign(b, { firmware: 'rover firmware', version: m[1], role: b.role || 'vehicle' });
    const r = line.match(/ board=(\S+)/); if (r) b.board = r[1];
    const a = line.match(/ mac=(\S+)/); if (a) b.mac = a[1];
    const fp = line.match(/ first_packet_ms=(\d+)/); if (fp) b.firstPacket = +fp[1];
  } else if ((m = line.match(/^OTA status .*table=(\S+) table_crc=([0-9a-fA-F]+)/))) Object.assign(b, { table: m[1], tableCrc: m[2].toLowerCase() });
  else if ((m = line.match(/Trail-Tracer-Rx-Monitor v(\S+) MAC (\S+)/))) {
    const rover = / role monitor/.test(line);
    Object.assign(b, { firmware: rover ? 'rover firmware' : 'Monitor firmware (old)', version: m[1], role: 'monitor', mac: m[2] });
  } else if (/^HB /.test(line)) { b.hb = true; if (!b.firmware) b.firmware = 'rover firmware'; if (!b.role) b.role = 'vehicle'; }
  else if ((m = line.match(/^STM ans .*cid=30r .*value=(\d+)/))) {
    const v = +m[1] >>> 0; b.stm = `app ${v & 255}.${(v >>> 8) & 255}.${(v >>> 16) & 255}`; b.stmLink = (v >>> 24) & 255;
  } else if (/^STM link v1/.test(line)) b.stm = 'app 1.x (link v1)';
}
async function readStm(b) {
  b.stm = null;
  await ask(b, 'stm read 30', /^STM ans .*cid=30r/, 3000);
  return b.stm || 'version not reported (an older STM32 app)';
}
async function detect(b) {
  b.detecting = true; render();
  let r = null;
  for (let i = 0; i < 8 && !r && !b.closing && !b.firmware; i++) r = await ask(b, 'boot', /^BOOT rover-firmware|Trail-Tracer-Rx-Monitor v/, 400);
  if (!r && !b.firmware) b.firmware = 'other firmware (no version report)';
  else if (b.role === 'monitor' && !b.version) await ask(b, 'V', /Trail-Tracer-Rx-Monitor v/, 1500);
  else if (b.firmware === 'rover firmware' && !b.version) await ask(b, 'boot', /^BOOT rover-firmware/, 1500);
  if (b.firmware === 'rover firmware' && b.role !== 'monitor' && !b.table) await ask(b, 'ota status', /^OTA status/, 2000);
  if (b.firmware === 'rover firmware' && b.role !== 'monitor' && !b.stm) b.stm = await readStm(b);
  b.detecting = false; render();
  if (expectRover && b.firmware !== 'rover firmware' && (b.redetect = (b.redetect || 0) + 1) <= 10 && !b.closing)
    waitLine(b, /./, 30000).then(l => { if (l && !b.closing && b.state === 'open' && !b.bridge) { say(`read the board again (${b.redetect})`); b.firmware = null; b.hb = false; detect(b); } });
  boardOpened(b);
}

function say(text) { if (job) { job.log.push(`${new Date().toTimeString().slice(0, 8)}  ${text}`); job.prog = false; } render(); }
const pageLog = [];
const usbId = port => { const i = port.getInfo(); return `${(i.usbVendorId || 0).toString(16).padStart(4, '0')}:${(i.usbProductId || 0).toString(16).padStart(4, '0')} (${i.usbProductId === ROM_PID ? 'update mode' : 'firmware runs'})`; };
function note(text) { pageLog.push(`${new Date().toTimeString().slice(0, 8)}  ${text}`); if (pageLog.length > 200) pageLog.shift(); render(); }
function progress(text) { if (!job) return; if (job.prog) job.log.pop(); say(text); job.prog = true; }
function step(id) {
  if (!job || !job.steps) return;
  const s = job.steps.find(x => x.id === id);
  if (!s || s.state === 'skip') return;
  for (const x of job.steps) if (x.state === 'now') x.state = 'done';
  s.state = 'now'; say(`STEP ${id}: ${s.text}`);
}
function stage(text, pct = null) { if (!job) return; job.stage = text; job.pct = pct; say('STEP: ' + text); }
function setPct(p) { if (job && job.pct !== p) { job.pct = p; render(); } }
async function run(name, fn, extra = {}) {
  if (job && !job.done) return;
  job = { name, log: [], done: false, stage: 'Starting', pct: null, result: null, ...extra };
  say(pageVer);
  say(`${name}: start`);
  try {
    const r = await fn(); job.result = { ok: true, text: typeof r === 'string' ? r : 'Done.' }; say(`${name}: done`);
    for (const x of job.steps || []) if (x.state !== 'skip') x.state = 'done';
    job.pct = 100;
  } catch (ex) {
    job.result = { ok: false, text: ex.message.includes(NO_LOADER) ? NO_LOADER : BUTTON_STEP };
    say(`${name}: STOPPED: ${ex.message}`); instruct('');
    for (const x of job.steps || []) if (x.state === 'now') x.state = 'fail';
  }
  job.done = true; if (!job.result.ok) job.pct = null; expectRover = false; render();
  for (const x of [...boards.values()]) if (x.state === 'open' && !x.rom) { x.connected = true; await releaseBoard(x).catch(() => {}); }
  for (const x of [...boards.values()]) if (x.rom) { const r = x.rom; x.rom = null; await romLinkClose(x, r); }
  await releaseAll();
  for (const x of [...boards.values()]) if (x.state === 'open') x.state = 'closed';
  render();
}
async function fetchBuild(build, path) {
  const f = build.m.files.find(x => x.path === path);
  if (!f) throw new Error(`${path} is not in the manifest`);
  const data = new Uint8Array(await (await fetch(build.base + path)).arrayBuffer());
  const sha = await sha256hex(data);
  if (sha !== f.sha256) throw new Error(`${path}: SHA-256 ${sha.slice(0, 12)} is not ${f.sha256.slice(0, 12)} (manifest)`);
  return data;
}
async function sha256hex(data) { return [...new Uint8Array(await crypto.subtle.digest('SHA-256', data))].map(x => x.toString(16).padStart(2, '0')).join(''); }
const CRC32 = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
function crc32(d) { let c = 0xFFFFFFFF; for (const x of d) c = CRC32[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; }
const hex = d => [...d].map(x => x.toString(16).padStart(2, '0')).join('');

async function otaUpdate(b0, build) {
  stage('Checking the firmware file');
  const data = await fetchBuild(build, build.m.esp32.usb_update.file);
  if (data[0] !== 0xE9 || (data[12] | data[13] << 8) !== 2 || new DataView(data.buffer).getUint32(0x20, true) !== 0xABCD5432) throw new Error('firmware.bin is not an ESP32-S2 app image');
  const sha = await sha256hex(data);
  let b = b0;
  const status = async () => { const r = await ask(b, 'ota status', /^OTA status/, 5000); return r; };
  step('esp');
  stage('Sending the new firmware to the board', 0);
  for (let attempt = 1; ; attempt++) {
    const r = await ask(b, `ota begin ${data.length} ${sha} confirm crc`, /^OTA (begin ok|error)/, 8000);
    say(`attempt ${attempt}: ${r}`);
    if (!r || !r.startsWith('OTA begin ok')) throw new Error('the board refused "ota begin"');
    const crc = /crc=1/.test(r);
    let off = 0, resends = 0, bad = false;
    while (off < data.length) {
      const chunk = data.subarray(off, off + 512);
      const line = `ota w ${off} ${hex(chunk)}` + (crc ? ` ${crc32(chunk).toString(16).padStart(8, '0')}` : '');
      const a = await ask(b, line, /^OTA (ok|error)/, 5000);
      if (a === `OTA ok ${off + chunk.length}`) { off += chunk.length; setPct(Math.floor(100 * off / data.length)); progress(`ESP32 ${Math.floor(100 * off / data.length)} %  (${off} of ${data.length} B)`); continue; }
      const st = await status(), w = st && /active=1/.test(st) ? +st.split('written=')[1].split(/\s/)[0] : null;
      if (w === off + chunk.length) { off = w; continue; }
      if (w === off && ++resends <= 20) { say(`line at ${off} not taken (${a}); sent again`); continue; }
      say(`answer at ${off}: ${a}; board count ${w}: abort, start again`); await send(b, 'ota abort'); bad = true; break;
    }
    if (!bad) break;
    if (attempt === 3) throw new Error('3 attempts failed');
  }
  step('start');
  stage('The board checks the new firmware and restarts');
  const end = await ask(b, 'ota end', /^OTA (done|error)/, 15000);
  say(String(end || 'no answer to "ota end" (the board can restart before it): checking'));
  if (end && !end.startsWith('OTA done')) throw new Error('"ota end" failed');
  const oldPort = b.port; await closeBoard(b);
  b = await waitBoard(x => x.mode === 'app' && x.port !== oldPort || x.t0 > Date.now() - 100000 && x.mode === 'app' && x.lines.some(l => /^BOOT rover-firmware/.test(l)), 100000,
    'The board restarts with the new firmware; the page opens it again.');
  if (!b) throw new Error('The board did not come back on USB within 100 s. It returns to its old firmware by itself after 2 min.');
  stage('Confirming the new firmware');
  let st = null;
  for (let i = 0; i < 6 && !st; i++) { st = await status(); if (!st) await waitLine(b, /./, 4000); }
  say(String(st));
  if (!st || !/state=1/.test(st)) throw new Error('The board did not start the new firmware. It still runs its old firmware.');
  say(String(await ask(b, 'ota confirm', /^OTA/, 5000)));
  say(String(await status()));
  say(`ESP32 now: ${b.firmware} ${b.version || ''}`);
  return `Installed. The ESP32 now runs ${b.firmware} ${b.version || ''}.`;
}

function nvsIapholdPage() {
  const crc = d => { let c = 0; for (const x of d) c = CRC32[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; };
  const p = new Uint8Array(4096).fill(0xFF), v = new DataView(p.buffer);
  v.setUint32(0, 0xFFFFFFFE, true); v.setUint32(4, 0, true); p[8] = 0xFE;
  v.setUint32(28, crc(p.subarray(4, 28)), true);
  p[32] = 0xFA;
  [[0, 'wfrx_bench'], [1, 'iaphold']].forEach(([ns, key], i) => {
    const e = p.subarray(64 + 32 * i, 96 + 32 * i);
    e.set([ns, 0x01, 1, 0xFF]); e.fill(0, 8, 24); e.set(enc.encode(key), 8); e[24] = 1;
    new DataView(e.buffer, e.byteOffset).setUint32(4, crc(Uint8Array.from([...e.subarray(0, 4), ...e.subarray(8, 32)])), true);
  });
  return p;
}
function partitionTable(table) {
  const out = [];
  for (let i = 0; i + 32 <= table.length; i += 32) {
    const v = new DataView(table.buffer, table.byteOffset + i);
    if (v.getUint16(0, true) !== 0x50AA) break;
    out.push({ type: table[i + 2], subtype: table[i + 3], offset: v.getUint32(4, true), size: v.getUint32(8, true),
      label: new TextDecoder().decode(table.subarray(i + 12, i + 28)).replace(/\0.*$/s, '') });
  }
  return out;
}
// The S2 ROM takes a 2 MB flash until the host sends its SPI parameters, and esptool-js 0.4.7 never sends them: every
// write above 0x200000 (drive 0x280000, logs 0x380000) did not land, and reads there fail (a test board, read with
// and without them). The page sends them itself: 4 MB, 64 KB blocks, 4 KB sectors, 256 B pages (esptool flash_set_parameters).
async function romFlashSize(loader) {
  const pk = new Uint8Array(24), v = new DataView(pk.buffer);
  [0, 0x400000, 0x10000, 0x1000, 0x100, 0xFFFF].forEach((x, i) => v.setUint32(4 * i, x, true));
  await loader.checkCommand('set SPI params', 0x0B, pk);
}
async function romAttach(loader) { await loader.detectChip('no_reset'); await romFlashSize(loader); }
async function flashRead(loader, addr, len) {
  const out = new Uint8Array(len);
  for (let at = 0; at < len; at += 64) {
    const n = Math.min(64, len - at), pk = new Uint8Array(8), v = new DataView(pk.buffer);
    v.setUint32(0, addr + at, true); v.setUint32(4, n, true);
    const r = await loader.checkCommand('read flash block', 0x0E, pk);
    if (!r || r.length < n) throw new Error(`flash read at 0x${(addr + at).toString(16)}: ${r ? r.length : 0} of ${n} B`);
    out.set(r.subarray(0, n), at);
  }
  return out;
}
const cstr = (d, at, n) => new TextDecoder().decode(d.subarray(at, at + n)).replace(/\0.*$/s, '');
async function romIdentify(read, tailOf = buildTail) {
  const tab = [];
  for (let at = 0x8000; at < 0x8C00; at += 64) { const t = partitionTable(await read(at, 64)); tab.push(...t); if (t.length < 2) break; }
  const ota = tab.filter(q => q.type === 0 && q.subtype >= 0x10 && q.subtype < 0x20).sort((a, c) => a.subtype - c.subtype);
  const otadata = tab.find(q => q.type === 1 && q.subtype === 0);
  let slot = tab.find(q => q.type === 0 && q.subtype === 0) || ota[0];
  if (otadata && ota.length) {
    const seqs = [];
    for (const off of [0, 0x1000]) { const seq = new DataView((await read(otadata.offset + off, 4)).buffer).getUint32(0, true); if (seq && seq !== 0xFFFFFFFF) seqs.push(seq); }
    if (seqs.length) slot = ota[(Math.max(...seqs) - 1) % ota.length];
  }
  if (!slot) return { text: 'ESP32 firmware: none in flash', detail: 'no app partition' };
  const h = await read(slot.offset, 0xB0);
  if (h[0] !== 0xE9) return { text: 'ESP32 firmware: none in flash', detail: `no image at 0x${slot.offset.toString(16)}` };
  const desc = new DataView(h.buffer).getUint32(0x20, true) === 0xABCD5432 ? { version: cstr(h, 0x30, 32), project: cstr(h, 0x50, 32), date: cstr(h, 0x80, 16) } : {};
  let off = slot.offset + 24;
  for (let i = 0; i < h[1]; i++) { const sh = i === 0 ? h.subarray(24, 32) : await read(off, 8); off += 8 + new DataView(sh.buffer, sh.byteOffset).getUint32(4, true); }
  const end = slot.offset + ((off - slot.offset + 16) & ~15);
  const sha = h[23] === 1 ? hex(await read(end, 32)) : null;
  const detail = `${slot.label} at 0x${slot.offset.toString(16)}: ${desc.project || '?'} ${desc.version || ''} ${desc.date || ''}`.trim();
  const hit = sha && await matchBuild(sha, tailOf);
  if (hit) return { version: hit.m.version, sha, detail, text: `ESP32 firmware ${hit.m.version}` };
  return { sha, detail, text: 'ESP32 firmware: V1.0 or other' };
}
const tails = new Map();
async function buildTail(url) {
  if (tails.has(url)) return tails.get(url);
  let t = null;
  try {
    const r = await fetch(url, { headers: { Range: 'bytes=-32' } }), d = new Uint8Array(await r.arrayBuffer());
    if (d.length >= 32) t = hex(r.status === 206 ? d.subarray(0, 32) : d.subarray(d.length - 32));
  } catch { /* not reachable: no match */ }
  tails.set(url, t);
  return t;
}
async function matchBuild(sha, tailOf) {
  for (const x of allBuilds.filter(x => x.m.chip === 'esp32s2' && x.m.esp32 && x.m.esp32.full_load)) {
    const parts = x.m.esp32.full_load.parts || [], app = parts.find(p => /firmware/.test(p.file)) || parts.find(p => parseInt(p.offset, 16) === 0x10000);
    if (app && await tailOf(x.base + app.file) === sha) return x;
  }
  return null;
}
async function romRead(b) {
  b.detecting = true; render();
  const { ESPLoader, Transport } = await loadEsptool();
  const transport = new Transport(b.port, true);
  try {
    const loader = new ESPLoader({ transport, baudrate: 115200, romBaudrate: 115200, terminal: { clean() {}, writeLine() {}, write() {} } });
    await romAttach(loader);
    b.rom = { transport, loader };
    b.flash = await romIdentify((a, n) => flashRead(loader, a, n));
    b.lines.push(`flash: ${b.flash.detail}${b.flash.sha ? `, SHA-256 ${b.flash.sha.slice(0, 12)}` : ''}`);
    b.state = 'open';
  } catch (ex) {
    b.flash = { text: 'ESP32 firmware: not read', detail: ex.message }; b.lines.push(`flash read: ${ex.message}`);
    b.rom = null; await transport.disconnect().catch(() => {}); b.state = 'closed';
  } finally { b.detecting = false; render(); }
}
async function romLinkClose(b, r) {
  if (!b.port.gone) { await Promise.race([watchdogReset(r.loader).catch(() => {}), sleep(1000)]); await sleep(150); }
  await r.transport.disconnect().catch(() => {});
}
async function watchdogReset(loader) {
  say('watchdog reset (esptool watchdog_reset)');
  await loader.writeReg(0x3F4080AC, 0x50D83AA1);
  await loader.writeReg(0x3F408098, 2000);
  await loader.writeReg(0x3F408094, ((1 << 31) | (5 << 28) | (1 << 8) | 2) >>> 0);
  await loader.writeReg(0x3F4080AC, 0).catch(() => {});
}
// A restart with a USB detach first, as the rover firmware's usb_guard::restart() does ("so the host sees the board
// leave", 300 ms): without it the S2 restarts with its USB pull-up still on, the PC sees no removal, and the new
// firmware's USB device often never appears until a replug (a test board: 3 of 5 runs). The RTC watchdog is armed first with
// about 0.5 s (90 kHz slow clock), then the ROM's USB is detached (USB OTG DCTL soft disconnect); the reset ends the
// detach. The detach write gets no answer: the USB is gone.
async function watchdogResetDetached(loader) {
  say('watchdog reset with a USB detach first');
  await loader.writeReg(0x3F4080AC, 0x50D83AA1);
  await loader.writeReg(0x3F408098, 45000);
  await loader.writeReg(0x3F408094, ((1 << 31) | (5 << 28) | (1 << 8) | 2) >>> 0);
  await loader.writeReg(0x3F4080AC, 0).catch(() => {});
  const DCTL = 0x60080804, dctl = await loader.readReg(DCTL).catch(() => null);
  if (dctl == null) { say('USB DCTL not read: restart without the detach'); return; }
  await Promise.race([loader.writeReg(DCTL, (dctl | 2) >>> 0).catch(() => {}), sleep(300)]);
}
async function withLoader(b, fn) {
  const { ESPLoader, Transport } = await loadEsptool();
  const term = { clean() {}, writeLine: t => say('esptool: ' + t), write: t => { if (t.trim()) say('esptool: ' + t.trim()); } };
  if (b.port.readable) await b.port.close().catch(() => {});
  const transport = new Transport(b.port, true);
  try {
    const loader = new ESPLoader({ transport, baudrate: 115200, romBaudrate: 115200, terminal: term, debugLogging: true });
    await romAttach(loader);
    return await fn(loader, transport);
  } finally { await transport.disconnect().catch(() => {}); if (b.port.readable) await b.port.close().catch(() => {}); boards.delete(b.port); render(); }
}
let mergedBridge = null;
const S2 = { STRAP: 0x3F404038, SPI_BOOT: 0x8, OPTION1: 0x3F408128, FORCE_DL: 0x1 };
async function romLoad(b0, build, erase, stmBuild = null) {
  stage('Checking the firmware files');
  const fl = build ? build.m.esp32.full_load : { parts: [] }, blobs = [];
  for (const x of fl.parts) { blobs.push({ data: await fetchBuild(build, x.file), address: parseInt(x.offset, 16) }); say(`${x.file} at ${x.offset}: ${blobs.at(-1).data.length} B, SHA-256 checked`); }
  const fillRest = () => {
    const SECTOR = 0x1000, SIZE = 0x400000, used = blobs.map(x => [Math.floor(x.address / SECTOR) * SECTOR, Math.ceil((x.address + x.data.length) / SECTOR) * SECTOR]).sort((a, c) => a[0] - c[0]);
    let at = 0;
    for (const [s, e] of [...used, [SIZE, SIZE]]) {
      if (s > at) { blobs.unshift({ data: new Uint8Array(s - at).fill(0xFF), address: at }); say(`erase 0x${at.toString(16)} to 0x${s.toString(16)}`); }
      at = Math.max(at, e);
    }
  };
  let b = b0;
  if (b.mode !== 'rom') {
    step('loader');
    stage('Preparing the board');
    const oldPort = b.port;
    if (b.firmware === 'rover firmware') {
      const a = await ask(b, 'dld', /^(BENCH restart into ROM|dld refused)/, 1500);
      say(`"dld": ${a || 'no answer'}`);
      if (a && /refused/.test(a)) throw plain('The vehicle moves, so the board refuses the install. Stop the vehicle, then start the install again.');
      await closeBoard(b).catch(() => {}); b = null;
    }
    else {
      await closeBoard(b).catch(() => {});
      say('1200-baud touch');
      try { await oldPort.open({ baudRate: 1200 }); await oldPort.setSignals({ dataTerminalReady: false }).catch(() => {}); await oldPort.close(); }
      catch (ex) { say(`1200-baud touch: ${ex.message}`); }
      b = await waitBoard(x => x.mode === 'rom', 8000, 'The board restarts into ROM download mode.');
      if (!b && !oldPort.gone) {
        say('no ROM download mode 8 s after the touch: USB reset');
        try {
          await oldPort.open({ baudRate: 115200 });
          for (const [d, r] of [[false, true], [true, true], [true, false], [false, false]]) { await oldPort.setSignals({ dataTerminalReady: d, requestToSend: r }).catch(() => {}); await sleep(60); }
          await oldPort.close();
        } catch (ex) { say(`USB reset: ${ex.message}`); }
      }
    }
    stage('Preparing the board');
    if (!b) b = await waitBoardHint(x => x.mode === 'rom', 60000, '"ESP32-S2"');
    if (!b) {
      stage('Waiting for you');
      say('no ROM download mode by itself: the button');
      b = await personPowerCycle(x => x.mode === 'rom');
    }
    instruct('');
    if (b.port === oldPort) say('same port object for the load');
  }
  step('esp');
  stage('Connecting to the board');
  const { ESPLoader, Transport } = await loadEsptool();
  const term = { clean() {}, writeLine: t => say('esptool: ' + t), write: t => { if (t.trim()) say('esptool: ' + t.trim()); } };
  const kept = b.rom; b.rom = null;
  if (!kept && b.port.readable) await b.port.close().catch(() => {});
  const transport = kept ? kept.transport : new Transport(b.port, true);
  let entry = null, merged = false;
  try {
  const loader = kept ? kept.loader : new ESPLoader({ transport, baudrate: 115200, romBaudrate: 115200, terminal: term, debugLogging: true });
  if (kept) say('the update-mode link of Connect'); else await romAttach(loader);
  if (!b.flash) {
    try { b.flash = await romIdentify((a, n) => flashRead(loader, a, n)); say(`ESP32 firmware in flash: ${b.flash.text} (${b.flash.detail})`); }
    catch (ex) { say(`flash read: ${ex.message}`); }
    if (b0 !== b) b0.flash = b.flash;
    render();
  }
  say(`chip: ${await loader.chip.getChipDescription(loader)}`);
  const strap = await loader.readReg(S2.STRAP), opt1 = await loader.readReg(S2.OPTION1);
  const byButton = (strap & S2.SPI_BOOT) === 0 && (opt1 & S2.FORCE_DL) === 0;
  say(`download mode entry: ${byButton ? 'button "0" (watchdog reset)' : 'firmware (force-download bit)'}; strap 0x${(strap >>> 0).toString(16)}, option1 0x${(opt1 >>> 0).toString(16)}`);
  if (byButton) {
    const held = async () => ((await loader.readReg(0x3F40403C).catch(() => 1)) & 1) === 0;
    if (await held()) {
      instruct('Let go of the button.');
      for (let i = 0; i < 200 && await held(); i++) await sleep(300);
      instruct('');
    }
  }
  const tb = blobs.find(x => x.address === 0x8000), table = tb ? partitionTable(tb.data) : [];
  if (build) {
    for (const q of table.filter(q => q.type === 0x01)) {
      if (blobs.some(x => x.address === q.offset)) continue;
      if (q.subtype === 0x02 && stmBuild && byButton) {
        blobs.push({ data: nvsIapholdPage(), address: q.offset });
        if (q.size > 4096) blobs.push({ data: new Uint8Array(q.size - 4096).fill(0xFF), address: q.offset + 4096 });
        merged = true; say(`${q.label} at 0x${q.offset.toString(16)}: iaphold = 1, so the first start of the new firmware opens the STM32 bridge`);
      } else { blobs.push({ data: new Uint8Array(q.size).fill(0xFF), address: q.offset }); say(`${q.label} at 0x${q.offset.toString(16)}: overwritten (${q.size} B)`); }
    }
  }
  // V1 factory stage: the ROM does not erase a region written with 0xFF only (a test board: the rover's drive and
  // settings stayed under V1). Zeros make V1 find no drive and no settings: it formats the drive and writes its defaults.
  // Settings 0x9000 +20 KB (phy_init at 0xe000 takes the build's own 0xe000 part); the drive's first 64 KB at 0x280000.
  const ZEROS = [[0x9000, 0x5000], [0x280000, 0x10000]];
  if (factoryFinal) for (const [za, zn] of ZEROS) {
    const x = blobs.find(y => y.address === za && y.data.every(v => v === 255));
    if (!x) throw new Error(`factory: no 0xFF region at 0x${za.toString(16)} to clear`);
    x.data = new Uint8Array(x.data.length).fill(0xFF); x.data.fill(0, 0, zn); say(`factory: zeros at 0x${za.toString(16)} +${zn} B`);
  }
  if (erase) fillRest();
  await romFlashSize(loader);                           // also on the kept link of Connect
  loader.FLASH_WRITE_SIZE = 0x400; if (loader.chip) loader.chip.FLASH_WRITE_SIZE = 0x400;
  const sizes = blobs.map(x => x.data.length), total = sizes.reduce((a, c) => a + c, 0);
  stage('Writing the ESP32 firmware', 0);
  const bstr = d => { let s = ''; for (let i = 0; i < d.length; i += 0x8000) s += String.fromCharCode.apply(null, d.subarray(i, i + 0x8000)); return s; };
  await loader.writeFlash({ fileArray: blobs.map(x => ({ data: bstr(x.data), address: x.address })), flashSize: '4MB', flashMode: 'dio', flashFreq: '80m',
    eraseAll: false, compress: true, reportProgress: (i, w, t) => { setPct(Math.floor(100 * (sizes.slice(0, i).reduce((a, c) => a + c, 0) + sizes[i] * w / t) / total)); progress(`file ${i + 1} of ${blobs.length}: ${Math.floor(100 * w / t)} %`); } });
  if (factoryFinal) for (const [za, zn] of ZEROS) for (const at of [za, za + zn - 4096]) {
    const d = await flashRead(loader, at, 4096);
    if (!d.every(v => v === 0)) throw new Error(`factory: 0x${at.toString(16)} does not read back as zeros`);
    say(`factory: 0x${at.toString(16)} +4 KB reads back as zeros`);
  }
  if (build) step('start');
  if (build) {
    stage('Starting the new firmware');
    await loader.writeReg(S2.OPTION1, 0, S2.FORCE_DL).catch(ex => say(`force-download bit not cleared: ${ex.message}`));
    if (FACTORY) await watchdogResetDetached(loader); else await watchdogReset(loader);   // factory first; rover page after a test
    await sleep(150);
  }
  entry = byButton ? 'button' : 'firmware';
  } finally { await transport.disconnect().catch(() => {}); if (b.port.readable) await b.port.close().catch(() => {}); }
  boards.delete(b.port); render();
  if (!build) return 'The whole ESP32 is erased. It runs no firmware and stays in update mode until the next update.';
  if (factoryFinal || factoryHelper) return 'Written and started.';
  const rover = build.m.esp32.usb_update, back = x => x.mode === 'app' && !x.detecting && (rover ? x.firmware === 'rover firmware' && !!x.version : !!x.firmware);
  b = null;
  expectRover = !!rover;
  const appTest = merged ? (x => x.mode === 'app' && x.bridge) : back;
  stage('The board restarts with the new firmware');
  if (merged) bridgeNext = true;
  try {
    // factory mode: the S2 seldom comes back on USB after a ROM write (a test board: 3 of 4 runs), so the replug comes after 12 s
    for (let i = 0; i < (FACTORY ? 1 : 2) && !b; i++) {
      const nb = await waitBoardHint(x => appTest(x) || x.mode === 'rom', FACTORY ? 12000 : 45000, 'the board');
      if (nb && nb.mode === 'rom') {
        say('the board came back in ROM download mode');

        await withLoader(nb, async (loader, tr) => {
          const held = async () => ((await loader.readReg(0x3F40403C).catch(() => 1)) & 1) === 0;
          if (await held()) { instruct('Let go of the button.'); for (let i = 0; i < 400 && await held(); i++) await sleep(150); instruct(''); }
          await loader.writeReg(S2.OPTION1, 0, S2.FORCE_DL).catch(() => {});
          await watchdogReset(loader); await sleep(150);
        });
        continue;
      }
      b = nb;
    }
  } finally { bridgeNext = false; }
  if (b && merged) { expectRover = false; mergedBridge = b; return 'Installed. The ESP32 firmware is written.'; }
  if (!b && merged) {
    say('no new firmware after the reset: the fallback, a power cycle');
    bridgeNext = true;
    stage('Waiting for you');
    try { mergedBridge = await personPowerCycle(x => x.mode === 'app' && x.bridge, REPLUG_STEP); } finally { bridgeNext = false; }
    return 'Installed. The ESP32 firmware is written.';
  }
  if (!b) {
    say('no new firmware after the reset: the fallback, a power cycle');
    stage('Waiting for you');
    b = await personPowerCycle(back, REPLUG_STEP);
  }
  expectRover = false;
  say(`ESP32 now: ${b.firmware} ${b.version || ''}`);
  return rover ? `Installed. The ESP32 now runs ${b.firmware} ${b.version || ''}.` : `Installed. The ESP32 now runs ${b.firmware}.`;
}

function crc16(d) { let c = 0xFFFF; for (const x of d) { c ^= x << 8; for (let k = 0; k < 8; k++) c = c & 0x8000 ? ((c << 1) ^ 0x8005) & 0xFFFF : (c << 1) & 0xFFFF; } return c; }
function frame(cid, payload = new Uint8Array(0)) {
  const f = new Uint8Array(6 + payload.length), v = new DataView(f.buffer);
  v.setUint16(0, 6 + payload.length, true); v.setUint16(2, cid, true); f.set(payload, 4);
  v.setUint16(4 + payload.length, crc16(f.subarray(0, 4 + payload.length)), true);
  return f;
}
async function readFrame(b, ms) {
  const end = Date.now() + ms;
  for (;;) {
    while (b.bin.length >= 4) {
      const v = new DataView(b.bin.buffer, b.bin.byteOffset), len = v.getUint16(0, true), cid = v.getUint16(2, true);
      if (len < 6 || len > 1100 || cid < 1 || cid > 5) { b.bin = b.bin.slice(1); b.skipped = (b.skipped || 0) + 1; continue; }
      if (b.bin.length < len) break;
      const f = b.bin.slice(0, len), crc = new DataView(f.buffer).getUint16(len - 2, true);
      if (crc !== crc16(f.subarray(0, len - 2))) { b.bin = b.bin.slice(1); b.skipped = (b.skipped || 0) + 1; continue; }
      b.bin = b.bin.slice(len);
      if (b.skipped) { say(`${b.skipped} bytes before the IAP frame skipped (ESP32 text)`); b.skipped = 0; }
      return { cid, p: f.slice(4, len - 2) };
    }
    const left = end - Date.now();
    if (left <= 0) throw new Error(`no IAP answer in ${ms} ms`);
    await new Promise(r => { b.binWait = r; setTimeout(r, Math.min(left, 200)); });
  }
}
async function stmLoad(b0, build) {
  step('stm');
  stage('Checking the STM32 files');
  const header = await fetchBuild(build, build.m.stm32.update.header);
  const image = await fetchBuild(build, build.m.stm32.update.image);
  const hv = new DataView(header.buffer), n = image.length / 1040;
  if (header.length !== 52 || image.length % 1040 || n !== hv.getUint16(18, true)) throw new Error(`header ${header.length} B, image ${image.length} B, chunk count ${hv.getUint16(18, true)}`);
  say(`STM32 image: ${n} chunks`);
  let b = b0;
  stage('Preparing the board for the STM32 update');
  const set = await ask(b, 'stm iaphold', /STM iaphold (set|:)/, 10000);
  say(String(set));
  if (set && !/STM iaphold set/.test(set)) throw new Error(set);
  if (!set) throw new Error('no "STM iaphold set" (rover firmware 0.5.26 or later needed)');
  const oldPort = b.port; await closeBoard(b);
  bridgeNext = true;
  stage('Waiting for you');
  try {
    b = await personPowerCycle(x => x.mode === 'app' && x.port !== oldPort && x.bridge, REPLUG_STEP);
  } finally { bridgeNext = false; }
  return stmBridge(b, build, header, image);
}
const NO_LOADER = 'The Rx STM32 cannot take an update over USB. Nothing was written to it. Contact support.';
async function stmBridge(b, build, header = null, image = null) {
  step('stm');
  if (!header) {
    header = await fetchBuild(build, build.m.stm32.update.header);
    image = await fetchBuild(build, build.m.stm32.update.image);
  }
  const n = image.length / 1040;
  const dec = new TextDecoder(), txt = dec.decode(b.bin), k = txt.indexOf('no text until it ends');
  if (k >= 0) {
    const line = txt.slice(txt.lastIndexOf('STM bridge on', k), (txt.indexOf('\n', k) + 1 || txt.length)).trim();
    say(line);
    if (/HELLO not answered/.test(line)) { say('no HELLO answer (bridge line)'); throw plain(NO_LOADER); }
  } else say(b.restartLine ? 'the bridge of "stm restart" is on: HELLO at once' : 'no "STM bridge on" line yet: HELLO at once');
  b.restartLine = null;
  stage('Checking the STM32');
  await send(b, frame(1));
  const h = await readFrame(b, 2000).then(f => f.cid === 1 && f.p.length >= 7 ? f : Promise.reject(new Error('not a HELLO answer'))).catch(() => {
    if (/BOOT rover-firmware/.test(dec.decode(b.bin))) throw new Error('the board started normally, not as an STM32 bridge: the update flag was gone at this power-on');
    say('no HELLO answer'); throw plain(NO_LOADER);
  });
  say(`HELLO: boot status ${h.p[0]}, app ${h.p[1]}.${h.p[2]}.${h.p[3]}, IAP ${h.p[4]}.${h.p[5]}.${h.p[6]}`);
  let r;
  await send(b, frame(2, header));
  r = await readFrame(b, 2000);
  if (r.cid !== 2 || r.p[0]) throw new Error(`BEGIN_UPDATE refused: answer cid ${r.cid}, status ${r.p[0]}`);
  stage('Writing the STM32 app', 0);
  for (let i = 0; i < n; i++) {
    const pl = new Uint8Array(2 + 1040); new DataView(pl.buffer).setUint16(0, i, true); pl.set(image.subarray(i * 1040, (i + 1) * 1040), 2);
    await send(b, frame(3, pl));
    r = await readFrame(b, 6000);
    const idx = r.p[0] | r.p[1] << 8;
    if (r.cid !== 3 || idx !== i || r.p[2]) throw new Error(`chunk ${i}: answer cid ${r.cid}, echoed ${idx}, reason ${r.p[2]}`);
    setPct(Math.floor(100 * (i + 1) / n)); progress(`STM32 chunk ${i + 1} of ${n}`);
  }
  stage('Starting the new STM32 app', 100);
  await send(b, frame(4));
  r = await readFrame(b, 3000);
  if (r.cid === 1) { say(`boot status after FINALIZE: ${r.p[0]}, app ${r.p[1]}.${r.p[2]}.${r.p[3]}`); r = await readFrame(b, 3000); }
  if (r.cid !== 4 || r.p[0]) throw new Error(`FINALIZE failed: answer cid ${r.cid}, status ${r.p[0]}`);
  await send(b, frame(5));
  r = await readFrame(b, 3000);
  if (r.cid !== 5 || r.p[0]) throw new Error(`BOOT failed: answer cid ${r.cid}, status ${r.p[0]}`);
  say('STM32 app loaded and started. The ESP32 leaves the bridge 10 s after the last byte.');
  step('check');
  stage('Rx STM32 written. The Rx restarts (10 s)', 100);
  b.binMode = false; b.buf = '';
  if (!await waitLine(b, /STM bridge off/, 30000)) say('no "STM bridge off" line within 30 s');
  stage('Reading the new versions', 100);
  b.bridge = false;
  Object.assign(b, { firmware: null, version: null, role: null, stm: null, stmLink: null, table: null, tableCrc: null, hb: false });
  await detect(b);
  say(`ESP32 now: ${b.firmware} ${b.version || ''}; STM32 now: ${b.stm}`);
  return /^app /.test(b.stm) ? `Installed. The STM32 now runs ${b.stm}.` : 'Installed and started. The board did not report the STM32 version yet.';
}

const setHtml = (el, html) => { if (el._html !== html) { el._html = html; el.innerHTML = html; return true; } return false; };
const setText = (el, t) => { if (el.textContent !== t) el.textContent = t; };
function setLog(details, pre, text) {
  if (!details.open || pre._text === text) return;
  const atEnd = pre.scrollHeight - pre.scrollTop - pre.clientHeight < 4;
  pre._text = text; pre.textContent = text;
  if (atEnd) pre.scrollTop = pre.scrollHeight;
}
let lastDetail = ['', '', ''];
let serialOk = true, serialMsg = '', buildsLoaded = false, ready1 = false;
const hhmm = t => new Date(t).toTimeString().slice(0, 5);
const vcol = v => String(v == null ? '?' : v).padEnd(8);
const row = (label, v) => `${label.padEnd(8)} ${v}`;
const espVal = b => b.mode === 'rom' ? (b.flash ? b.flash.version || b.flash.text.replace(/^ESP32 firmware:? ?/, '') : '')
  : b.firmware === 'rover firmware' || b.role === 'monitor' ? b.version || '?'
  : b.firmware === 'Monitor firmware (old)' ? `${vcol(b.version)} old Monitor`
  : b.firmware ? 'V1.0 or other' : '';
const espLine = b => espVal(b) ? row('ESP32', espVal(b)) : '';
const stmLine = b => row('STM32', b.mode === 'rom' ? 'after update'
  : /^app \d/.test(b.stm || '') ? b.stm.slice(4) : /^app 1\.x/.test(b.stm || '') ? '1.x'
  : b.stm ? 'not reported' : 'not readable');
function offerFor(b) {
  if (!b || !b.connected || b.detecting || !b.firmware) return {};
  const o = offersFor(b);
  return { offer: o[0], missing: o.missing, refused: !o.length && o.refused && o.refused.length ? o.refused[0] : null, erase: o.erase };
}
function render() {
  if (!$('rxDetail')) return;
  const l = listed(), b = selected(), k = 1, running = !!(job && !job.done);
  let L = ['', '', ''];
  if (!serialOk) L = ['', '', ''];
  else if (running && !b) L = ['Rx unplugged', 'The page waits for the Rx.', ''];
  else if (b) {
    const name = k ? 'Rx' : '', rev = b.board && b.board !== 'RX_REVA' ? `, revision ${b.board.replace('RX_REV', '')}` : '';
    const e0 = job && job.esp && job.esp[0] === '?' && b.flash && b.flash.version ? b.flash.version : job && job.esp && job.esp[0];
    if (running && job.esp) L = [b.mode === 'rom' ? `${name} in update mode` : `${name} connected`, row('ESP32', `${vcol(e0)} → ${job.esp[1]}`), job.stm ? row('STM32', `${vcol(job.stm[0])} → ${job.stm[1]}`) : stmLine(b)];
    else if (!b.connected) L = [b.mode === 'rom' ? `${name} in update mode` : `${name} found, not connected`, '', ''];
    else if (b.detecting) L = [`${name} connected`, 'Reading the versions…', ''];
    else L = [(b.mode === 'rom' ? `${name} in update mode` : `${name} connected`) + rev, espLine(b), stmLine(b)];
  }
  const ended = !!(job && job.done);
  if (ended && !(b && b.connected)) L = lastDetail; else if (b) lastDetail = L;
  const detail = (running && !!b) || !!(b && (b.connected || b.detecting)) || ended;
  const o0 = offerFor(b), connected = !!(b && b.connected && !b.detecting);
  $('s2').classList.toggle('off', !(ready1 || running || connected));
  $('fwCard').classList.toggle('off', running);          // the firmware list reads with no Rx and no USB access in the browser
  setText($('s1say'), serialOk || !isSecureContext ? 'Hold the button while you plug in USB' : serialMsg);
  $('s1say').classList.toggle('nousb', !serialOk && isSecureContext);
  $('s4').classList.toggle('off', !(running || (job && job.result) || (connected && o0.offer && currentRef())));
  $('doneBtn').hidden = ready1; $('doneBtn').disabled = !serialOk;
  $('findBtn').hidden = detail; $('rxBtn').hidden = !detail;
  $('findBtn').disabled = !serialOk || !(ready1 || running);
  setHtml($('rxDetail'), detail ? L.map((t, i) => `<div class="line${i ? '' : ' big'}" title="${esc(t)}">${esc(t)}${i === 1 && b && b.detecting ? '<span class="spin"></span>' : ''}</div>`).join('') : '');
  $('rxBtn').disabled = running || !!(b && b.detecting);
  drawRefMenu();
  $('refBtn').disabled = running;
  const o = offerFor(b);
  let I = '', tone = '', arrows = false;
  if (!serialOk) { I = isSecureContext ? '' : serialMsg; tone = I ? 'bad' : ''; }   // no Web Serial: the one line is in the Rx card
  else if (running) { I = stepText || 'Keep the Rx plugged in.'; tone = stepText ? 'warn' : ''; arrows = stepText.startsWith(BUTTON_STEP); }
  else if (job && job.result) {
    if (job.result.ok) { I = job.kind === 'erase' ? 'ESP32 erased. Press Disconnect.' : '✓ Update done. Press Disconnect.'; tone = 'good'; }
    else if (job.result.text === NO_LOADER) { I = NO_LOADER; tone = 'bad'; }
    else if (job.steps && job.steps.some(x => x.id === 'esp' && x.state === 'done')) { I = 'Stopped. Plug in the Rx again, then press Find Rx Boards.'; tone = 'bad'; }
    else { I = `Stopped. ${CONNECT_STEP}, then press Find Rx Boards.`; tone = 'bad'; arrows = true; }
  }
  else if (!b) { I = ''; arrows = !ready1; }
  else if (b.error) { I = b.error; tone = 'bad'; arrows = true; }
  else if (!b.connected) { I = ''; arrows = !ready1; }
  else if (b.detecting) I = 'Reading the Rx…';
  else if (o.missing) { I = o.missing; tone = 'bad'; }
  else if (o.refused) { I = `This firmware does not fit this Rx (revision ${(b.board || 'RX_REVA').replace('RX_REV', '')}). Press Change.`; tone = 'bad'; }
  else if (!currentRef()) I = '';
  else if (!o.offer) { I = 'No firmware fits this Rx. Press Change.'; tone = 'bad'; }
  else I = 'Press Update. About 2 minutes.';
  setText($('instr'), I); $('instr').className = `instr ${tone}`;
  $('arrows').hidden = !arrows;
  $('updBtn').disabled = !serialOk || running || !o.offer || !!o.missing;
  const okEnd = !!(job && job.done && job.result && job.result.ok);
  $('s4').classList.toggle('ok', okEnd);
  setText($('updBtn'), okEnd ? 'Update again' : 'Update');
  $('rxBtn2').hidden = !okEnd;
  const pct = job ? job.pct : null, st = id => job && job.steps && job.steps.find(x => x.id === id);
  const now = job && job.steps && job.steps.find(x => x.state === 'now');
  if (job && !job.bars) job.bars = { Esp: 0, Stm: 0 };
  const hasStm = !!(st('stm') && st('stm').state !== 'skip');
  const rowOf = x => !x ? null : x.id === 'stm' || (x.id === 'check' && hasStm) ? 'Stm' : 'Esp';
  const active = running ? rowOf(now) : null;
  if (active && pct != null && (now.id === 'esp' || now.id === 'stm')) job.bars[active] = pct;
  for (const k of ['Esp', 'Stm']) {
    const s = st(k === 'Esp' ? 'esp' : 'stm'), skip = !job ? false : k === 'Stm' && !hasStm;
    if (s && s.state === 'done') job.bars[k] = 100;
    const v = !job || skip ? 0 : job.bars[k];
    $(`bar${k}`).style.width = `${v}%`;
    $(`row${k}`).classList.toggle('skip', skip);
    $(`row${k}`).classList.toggle('fail', !!(s && s.state === 'fail'));
    $(`spin${k}`).hidden = !(active === k && (pct == null || pct === 100 || (now && now.id !== 'esp' && now.id !== 'stm')));
    setText($(`pct${k}`), job && !skip && (v || active === k) ? `${String(v).padStart(3, ' ')} %` : '');
  }
  const kv = b ? [['ESP32 firmware', b.mode === 'rom' ? (b.flash ? b.flash.detail || b.flash.text : 'not read') : `${b.firmware || '-'} ${b.version || ''}`],
    ['role', b.role || '-'], ['Rx STM32 app', b.stm || '-'], ['board revision', b.board || (b.firmware ? 'RX_REVA (not reported)' : '-')],
    ['radio address', b.mac || '-'], ['USB', b.state]] : [];
  setHtml($('dKv'), kv.map(([x, y]) => `<span class="k">${esc(x)}</span><span>${esc(y)}</span>`).join(''));
  $('eraseBtn').disabled = !serialOk || running || !b || !b.connected || !!b.detecting;
  setLog($('more'), $('dLog'), [...pageLog, ...(job ? job.log : [])].join('\n'));
  setLog($('more'), $('dUsb'), b ? b.lines.slice(-200).join('\n') : '');
  if (FACTORY) renderFactory(b, running);
}
let stepText = '';
function instruct(text) { stepText = text; if (text) say('USER: ' + text); render(); }
function startUpdate() {
  const b = selected(), o = offerFor(b);
  if (!b || !o.offer || (job && !job.done)) return;
  const from = [b.mode === 'rom' ? (b.flash && b.flash.version) || '?' : b.version || '?', /^app \d/.test(b.stm || '') ? b.stm.slice(4) : '?'];
  return run('Update', async () => { await ensureOpen(b); return o.offer.go(); }, { esp: [from[0], o.offer.espV], stm: o.offer.stmV ? [from[1], o.offer.stmV] : null });
}
function startErase() {
  const b = selected(), o = offerFor(b);
  if (!b || !o.erase || (job && !job.done)) return;
  if (!confirm('Erase the whole ESP32? It then runs no firmware until the next update.')) return;
  return run('Erase', async () => { await ensureOpen(b); return o.erase.go(); }, { kind: 'erase' });
}

function vcmp(a, b) { const x = String(a).split(/[.+ ]/).map(Number), y = String(b).split(/[.+ ]/).map(Number); for (let i = 0; i < 3; i++) if ((x[i] || 0) !== (y[i] || 0)) return (x[i] || 0) - (y[i] || 0); return 0; }
function change(from, to) { if (!from) return 'Replaces an unknown version.'; const c = vcmp(to, from); return c === 0 ? `Same version (${from}).` : c > 0 ? `Replaces ${from}.` : `Older than the board (${from} to ${to}).`; }
const srcLine = (m, rel) => rel && rel === m.source.branch
  ? `${m.source.repo.split('/').pop()}, commit ${m.source.commit.slice(0, 7)}, release ${rel}`
  : `${m.source.repo.split('/').pop()}, branch ${m.source.branch}, commit ${m.source.commit.slice(0, 7)}`
  + (rel ? `, release ${rel}` : rel === false ? ', development branch' : '');
let refSel = null;
function fresh() {
  job = null; ready1 = false; selPort = null; refSel = null; lastDetail = ['', '', ''];
  $('refList').hidden = true; render();
}
let refInfo = {};
let allBuilds = [];
function refFromUrl() {
  const q = new URLSearchParams(location.search), fw = q.get('fw');
  if (!fw || !fw.includes('/')) return null;
  const i = fw.indexOf('/'), short = fw.slice(0, i), ref = fw.slice(i + 1), commit = (q.get('commit') || '').toLowerCase();
  const b = allBuilds.find(x => x.m.source.repo.split('/').pop() === short && x.m.source.branch === ref);
  return b ? { repo: b.m.source.repo, ref, ...(/^[0-9a-f]{7,40}$/.test(commit) ? { commit } : {}) } : null;
}
function urlFor(sel, pin) {
  const u = new URL(location.href), enc = s => s.split('/').map(encodeURIComponent).join('/');
  u.search = `?fw=${enc(sel.repo.split('/').pop() + '/' + sel.ref)}` + (pin ? `&commit=${encodeURIComponent(pin)}` : '');
  return u.toString();
}
const pinnedOf = sel => allBuilds.filter(x => x.m.source.repo === sel.repo && x.m.source.branch === sel.ref && x.m.source.commit.startsWith(sel.commit));
const pinMissing = sel => sel !== '*' && !!sel.commit && !pinnedOf(sel).length;
function offerBuilds(sel) {
  if (sel === '*' || !sel.commit) return builds;
  return [...pinnedOf(sel), ...builds.filter(x => !(x.m.source.repo === sel.repo && x.m.source.branch === sel.ref))];
}
const stmOnly = repo => builds.filter(x => x.m.source.repo === repo).every(x => x.m.chip === 'stm32g070');
const repoName = repo => ({ Trail_Tracer_Mower: 'Trail Tracer Mower', 'Trail-Tracer': 'Trail Tracer V1.0' })[repo.split('/').pop()]
  || repo.split('/').pop().replace(/[_-]+/g, ' ');
const vtext = v => /^\d/.test(v) ? 'v' + v : v;
function refsOf() {
  const m = new Map();
  for (const x of builds) {
    const s = x.m.source;
    if (stmOnly(s.repo)) continue;
    const k = s.repo + ' ' + s.branch;
    const r = m.get(k) || { repo: s.repo, ref: s.branch, release: x.release, commit: s.commit, t: s.built_utc, parts: [] };
    if (s.built_utc > r.t) Object.assign(r, { commit: s.commit, t: s.built_utc });
    if (x.m.chip === 'esp32s2' && x.m.esp32.usb_update) r.ver = vtext(x.m.version);
    r.parts.push(x.m.chip === 'stm32g070' ? `Rx STM32 app ${vtext(x.m.version)}` : x.m.esp32.usb_update ? `ESP32 firmware ${vtext(x.m.version)}` : 'ESP32 firmware');
    m.set(k, r);
  }
  const out = [...m.values()].map(r => {
    const info = refInfo[r.repo + ' ' + r.ref] || {}, tag = info.tag !== undefined ? info.tag : r.release === r.ref;
    const kind = tag ? (/-test\.\d+$/.test(r.ref) ? 'test' : 'release') : 'dev';
    const tv = (r.ref.match(/v?(\d+\.\d+\.\d+)/) || [])[1];
    const repo = r.repo.split('/').pop();
    const title = (info.title && (info.title + (r.ver ? ' ' + r.ver : ''))) || (kind === 'release' ? `Release ${tv ? 'v' + tv : r.ref}` : kind === 'test' ? `Test release ${tv ? 'v' + tv : r.ref}`
      : r.ref === 'main' ? (repo === 'Trail_Tracer_Mower' ? 'Development build (latest, untested)' : repo === 'Trail-Tracer' ? 'Original V1.0 firmware line' : 'Development build')
      : `Test build: ${r.ref}`);
    return { ...r, kind, title, description: info.description || [...new Set(r.parts)].join(', '), git: `${r.ref}, ${r.commit.slice(0, 7)}` };
  });
  const rank = r => ({ release: 0, test: 1, dev: 2 })[r.kind] * 2 + (r.kind === 'dev' && r.ref !== 'main' ? 1 : 0);
  return out.sort((a, b) => a.repo.localeCompare(b.repo) || rank(a) - rank(b) || b.t.localeCompare(a.t));
}
const inSel = (x, sel) => sel === '*' || stmOnly(x.m.source.repo) || (x.m.source.repo === sel.repo && x.m.source.branch === sel.ref);
function currentRef() {
  if (refSel === '*') return '*';
  const refs = refsOf(), fromUrl = refFromUrl();
  if (fromUrl) return fromUrl;
  if (refSel && refs.some(r => r.repo === refSel.repo && r.ref === refSel.ref)) return refSel;
  return null;
}
const builtOn = t => { const d = new Date(t); return isNaN(d) ? '' : `${d.getUTCDate()} ${d.toLocaleString('en', { month: 'short', timeZone: 'UTC' })} ${d.getUTCFullYear()}`; };
const refName = r => r.ver && !/\d+\.\d+\.\d+/.test(r.title) ? `${r.title.replace(/ \(latest, untested\)/, '')} ${r.ver}` : r.title;
const refWhere = (r, sha = r.commit) => `${r.ref} ${sha.slice(0, 7)}`;
const refWhen = r => { const st = r.parts.find(x => x.startsWith('Rx STM32 app ')); return builtOn(r.t) + (st ? ` · STM32 ${st.slice(13)}` : ''); };
const optHtml = (r, on, latest) => `<button class="refopt${on ? ' on' : ''}" data-v="${esc(JSON.stringify({ repo: r.repo, ref: r.ref }))}"><b>${esc(refName(r))}${latest ? '<span class="latest">latest</span>' : ''}</b>`
  + `<span class="dim">${esc(refWhen(r))}</span><span class="gitref">${esc(refWhere(r))}</span></button>`;
function drawRefMenu() {
  const refs = refsOf(), sel = currentRef(), list = $('refList');
  let cur = sel && sel !== '*' && refs.find(r => r.repo === sel.repo && r.ref === sel.ref);
  let F = buildsLoaded ? (refs.length ? ['Choose the firmware', '', ''] : ['No firmware found', '', '']) : ['Loading the firmware list…', '', ''];
  if (!buildsLoaded) F = ['Loading the firmware list…', '', ''];
  else if (sel === '*') F = ['Every build (test page)', '', ''];
  else if (cur) {
    let l3 = refWhere(cur);
    if (sel.commit) {
      const pb = offerBuilds(sel).find(x => x.m.source.repo === sel.repo && x.m.source.branch === sel.ref);
      l3 = pb ? `${refWhere(cur, pb.m.source.commit)} (exact version)` : `${cur.ref} ${sel.commit.slice(0, 7)}: not on this site`;
    }
    F = [refName(cur), refWhen(cur), l3];
  }
  ['fwL1', 'fwL2', 'fwL3'].forEach((id, i) => { setText($(id), F[i]); $(id).title = F[i]; });
  $('refBtn').hidden = !refs.length || sel === '*'; $('refBtn').textContent = cur ? 'Change' : 'Choose';
  $('spinFw').hidden = buildsLoaded;
  if (cur) {
    const newest = builds.find(x => x.m.source.repo === cur.repo && x.m.source.branch === cur.ref);
    const pinC = sel.commit ? offerBuilds(sel).find(x => x.m.source.repo === cur.repo && x.m.source.branch === cur.ref) : newest;
    const link = urlFor(cur, pinC ? pinC.m.source.commit.slice(0, 12) : sel.commit);
    if ($('refLink').href !== link) $('refLink').href = link;
    if (!window.PROGRAM_TEST) { const want = urlFor(cur, sel.commit); if (location.href !== want) history.replaceState(null, '', want); }
  }
  const key = refs.map(r => [r.repo, r.ref, r.commit, r.title].join(' ')).join('|') + (cur ? cur.repo + cur.ref : '');
  if (list.dataset.key === key) return;
  list.dataset.key = key;
  const all = [...refs].sort((a, c) => c.t.localeCompare(a.t));
  list.innerHTML = all.map((r, i) => optHtml(r, cur && cur.repo === r.repo && cur.ref === r.ref, i === 0)).join('');
  for (const el of list.querySelectorAll('.refopt')) el.onclick = () => chooseRef(el.dataset.v);
}
function chooseRef(v) {
  refSel = JSON.parse(v); $('refList').hidden = true;
  if (!window.PROGRAM_TEST) history.replaceState(null, '', urlFor(refSel));
  if (job && job.done) job = null;
  render();
}

const named = (label, v, sep = ' ') => `${label.replace(/\s*\bv?\d+\.\d+\.\d+[^\s,()]*/g, '').trim()}${sep}${v}`;
const canRestart = b => b.firmware === 'rover firmware' && vcmp(b.version, '0.5.65') >= 0 && /^app \d/.test(b.stm || '') && vcmp(b.stm.slice(4), '2.0.39') >= 0;
async function stmPart(r1, stm) {
  const nb = [...boards.values()].find(x => x.mode === 'app' && x.firmware === 'rover firmware' && x.state === 'open');
  if (!nb) throw plain(`${r1} The Rx STM32 update did not start: the Rx is not on USB. Press Update again.`);
  stage('Reading the Rx STM32');
  const t0 = Date.now();
  while (Date.now() - t0 < 5000 && !nb.closing) {
    nb.firstPacket = undefined;
    await ask(nb, 'boot', /^BOOT rover-firmware/, 1500);
    if (nb.firstPacket > 0) break;
    await waitLine(nb, /^HB /, 1500);
  }
  const linked = nb.firstPacket > 0;
  say(`Rx STM32 link: ${linked ? `first packet ${nb.firstPacket} ms after the ESP32 start` : 'no Rx STM32 packet in 5 s: it waits in its boot checker'}`);
  if (linked) {
    nb.stm = await readStm(nb); say(`Rx STM32 version: ${nb.stm}`); render();
    if (canRestart(nb)) {
      try { return `${r1} ${await stmRestart(nb, stm)}`; }
      catch (ex) {
        if (ex.plain) throw plain(`The ESP32 firmware is installed. ${ex.message}`);
        say(`no-power-cycle path: ${ex.message}; the replug path`);
        const back = [...boards.values()].find(x => x.mode === 'app' && x.firmware === 'rover firmware' && x.state === 'open' && !x.bridge);
        if (!back) throw plain(BUTTON_STEP);
        back.binMode = false; back.bridge = false;
        return stmLoadOr(r1, back, stm);
      }
    }
    return stmLoadOr(r1, nb, stm);
  }
  try {
    const set = await ask(nb, 'stm iaphold', /STM iaphold (set|:)/, 10000);
    say(String(set));
    if (set && /STM iaphold set/.test(set)) {
      const oldPort = nb.port;
      bridgeNext = true;
      await send(nb, 'restart').catch(() => {});
      say('restart (ESP32 only): the new start opens the Rx STM32 bridge');
      await closeBoard(nb).catch(() => {});
      let bb;
      try { bb = await waitBoardHint(x => x.mode === 'app' && x.port !== oldPort && x.bridge, 30000, 'the board'); } finally { bridgeNext = false; }
      if (bb) {
        try { return `${r1} ${await stmBridge(bb, stm)}`; }
        catch (ex) {
          if (ex.message !== NO_LOADER) throw ex;
          say('no HELLO answer after the restart: the Rx STM32 runs its app; the replug path');
          bb.binMode = false; bb.buf = ''; bb.bridge = false;
          if (!await waitLine(bb, /STM bridge off/, 30000)) say('no "STM bridge off" line within 30 s');
          Object.assign(bb, { firmware: null, version: null, role: null, stm: null, stmLink: null, table: null, tableCrc: null, hb: false });
          await detect(bb);
        }
      } else say('no bridge after the restart: the replug path');
    }
  } catch (ex) {
    if (ex.plain) throw plain(`The ESP32 firmware is installed. ${ex.message}`);
    say(`Rx STM32 by restart: ${ex.message}; the replug path`);
  }
  const back = [...boards.values()].find(x => x.mode === 'app' && x.firmware === 'rover firmware' && x.state === 'open' && !x.bridge);
  if (!back) throw plain(BUTTON_STEP);
  return stmLoadOr(r1, back, stm);
}
async function stmRestart(b, build) {
  step('stm');
  stage('Checking the Rx STM32 files');
  const header = await fetchBuild(build, build.m.stm32.update.header), image = await fetchBuild(build, build.m.stm32.update.image);
  if (header.length !== 52 || image.length % 1040) throw new Error(`header ${header.length} B, image ${image.length} B`);
  const first = new Uint8Array(52 + 1040); first.set(header); first.set(image.subarray(0, 1040), 52);
  stage('Preparing the Rx STM32 for its update');
  const re = /^STM restart (held|refused|failed)/;
  let a = await ask(b, 'stm restart clear', re, 3000);
  if (!a || !/held 0 B/.test(a)) throw new Error(`"stm restart clear": ${a || 'no answer'}`);
  for (let off = 0; off < first.length; off += 512) {
    const n = Math.min(off + 512, first.length);
    a = await ask(b, `stm restart add ${hex(first.subarray(off, n))}`, re, 3000);
    if (!a || !new RegExp(`held ${n} B`).test(a)) throw new Error(`"stm restart add" up to ${n} B: ${a || 'no answer'}`);
  }
  const on = waitLine(b, /^STM bridge on/, 15000);
  a = await ask(b, 'stm restart go', /^STM restart(:| failed| refused)/, 5000);
  say(String(a || 'no answer to "stm restart go"'));
  if (!a || !/^STM restart:/.test(a)) throw new Error(`"stm restart go": ${a || 'no answer'}`);
  const line = await on;
  if (!line) throw new Error('no "STM bridge on" within 15 s of "stm restart go"');
  say(line);
  Object.assign(b, { binMode: true, bin: new Uint8Array(0), bridge: true, restartLine: line });
  return stmBridge(b, build, header, image);
}
async function stmLoadOr(r1, nb, stm) {
  try { return `${r1} ${await stmLoad(nb, stm)}`; }
  catch (ex) { say(`STM32 part: ${ex.message}`); throw plain(`The ESP32 firmware is installed. ${ex.plain ? ex.message : 'The STM32 update stopped. Start the install again.'}`); }
}
function offersFor(b) {
  const out = [], no = [], sel = currentRef(), rev = b.board || 'RX_REVA';
  const pool = sel ? offerBuilds(sel).filter(x => inSel(x, sel)) : [];
  const stmExtra = pool.filter(x => stmOnly(x.m.source.repo) && x.m.chip === 'stm32g070');
  const groups = new Map();
  for (const x of pool) {
    if (stmOnly(x.m.source.repo)) continue;
    const k = x.m.source.repo + ' ' + x.m.source.branch, g = groups.get(k) || {};
    if (x.m.chip === 'esp32s2' && !g.esp) g.esp = x;
    if (x.m.chip === 'stm32g070' && !g.stm) g.stm = x;
    groups.set(k, g);
  }
  for (const g of groups.values()) {
    if (!g.esp) continue;
    const m = g.esp.m, rover = !!m.esp32.usb_update, stm = rover ? (g.stm || stmExtra[0]) : null;
    const what = `${rover ? named(m.label || 'ESP32 rover firmware', m.version) : named(m.label || m.source.repo.split('/').pop() + ' firmware', m.version, ', ')}`
      + (stm ? ` + ${named(stm.m.label || 'Rx STM32 app', stm.m.version)}` : '') + ` (${srcLine(m, g.esp.release)})`;
    const bad = [g.esp, stm].filter(Boolean).find(x => !x.m.boards.includes(rev));
    if (bad) { no.push({ l1: 'Full install', l2: what, reason: `This firmware is for board revision ${bad.m.boards.join(', ')}. This board is revision ${rev}.` }); continue; }
    const rom = b.mode === 'rom';
    const steps = [{ id: 'loader', text: 'Prepare', skip: rom }, { id: 'esp', text: 'Write ESP32' },
      { id: 'stm', text: 'Write Rx STM32', skip: !stm }, { id: 'check', text: 'Check' }];
    out.push({ l1: 'Update', l2: what, steps, espV: m.version, stmV: stm ? stm.m.version : null,
      l3: stm ? 'Erases the whole ESP32, writes its firmware and the Rx STM32 app. Wipes all user settings and calibration.'
        : 'Erases the whole ESP32 and writes its firmware. Wipes all user settings.',
      go: () => fullInstall(b, g.esp, stm, steps) });
  }
  out.refused = no;
  out.missing = sel && pinMissing(sel) ? `This exact version (${sel.commit.slice(0, 7)}) is not on this site. Press Change.` : null;
  const esteps = [{ id: 'loader', text: 'Prepare', skip: b.mode === 'rom' }, { id: 'esp', text: 'Erase ESP32' }];
  out.erase = { l1: 'Erase ESP32', l2: 'Erases the whole ESP32 flash and writes no firmware.',
    l3: 'The board then stays in bootloader mode until a Full install.',
    steps: esteps, go: () => { if (job) job.steps = esteps.map(x => ({ ...x, state: x.skip ? 'skip' : 'wait' })); return romLoad(b, null, true); } };
  return out;
}
async function otaInstall(b, esp, stm, steps) {
  if (job && steps) job.steps = steps.map(x => ({ ...x, state: x.skip ? 'skip' : 'wait' }));
  const r1 = await otaUpdate(b, esp);
  if (!stm) { step('check'); return r1; }
  return stmPart(r1, stm);
}
async function fullInstall(b, esp, stm, steps) {
  if (job && steps) job.steps = steps.map(x => ({ ...x, state: x.skip ? 'skip' : 'wait' }));
  mergedBridge = null;
  const r1 = await romLoad(b, esp, true, stm);
  if (!stm) { step('check'); return r1; }
  if (mergedBridge) {
    const bb = mergedBridge; mergedBridge = null;
    try { return `${r1} ${await stmBridge(bb, stm)}`; }
    catch (ex) {
      if (ex.message !== NO_LOADER) { say(`STM32 part: ${ex.message}`); throw plain(`The ESP32 firmware is installed. ${ex.plain ? ex.message : 'The STM32 update stopped. Start the install again.'}`); }
      say('no HELLO answer after a watchdog start: the STM32 may run its app; the normal STM32 path follows');
      bb.binMode = false; bb.buf = ''; bb.bridge = false;
      if (!await waitLine(bb, /STM bridge off/, 30000)) say('no "STM bridge off" line within 30 s');
      Object.assign(bb, { firmware: null, version: null, role: null, stm: null, stmLink: null, table: null, tableCrc: null, hb: false });
      await detect(bb);
      return stmPart(r1, stm);
    }
  }
  if (false) {
    try { return ''; }
    catch (ex) { say(`STM32 part: ${ex.message}`); throw plain(`The ESP32 firmware is installed. ${ex.plain ? ex.message : 'The STM32 update stopped. Start the install again.'}`); }
  }
  return stmPart(r1, stm);
}

// ----- V1 factory install: one link (?factory), one press, from update mode only (the button
// held at plug-in). 1: the pinned rover build and the shipped Rx STM32 app 0.1.0 (factory/, signed and encrypted).
// 2: the rover firmware's update command takes the board back to update mode. 3: the whole ESP32 erased, V1 written.
// V1 cannot load the Rx STM32 and does not read the frames of app 2.x, so stage 1 runs on every board.
// Stage 1 passes when the rover firmware prints "STM link v1" (the 122 B frames of app 0.1.0).
const FQ = new URLSearchParams(location.search);
const FACTORY = true;                                  // this site holds only the V1 factory restore
// V1 comes from factory/ as bytes (the build of Trail-Tracer main 172f102, tested), not from the site build list:
// a CI rebuild never changes it, and the V1 repo needs no tag
const FACTORY_CFG = { rover: { repo: 'rctestflight/Trail_Tracer_Mower', ref: 'v0.7.15' } };
const PICK_ROM = 'ESP32-S2', PICK_APP = 'TinyUSB CDC';   // the names in the browser's list (Chrome on Windows)
// The factory screen is a vertical stepper. Its words are fixed; nothing else shows. The page waits on a person with no time limit. A progress bar shows only
// while the page writes. Step 4 starts by itself after step 3.
let factoryStm = null, factoryV1 = null, factoryFinal = false, factoryHelper = false;
const factoryBuild = sel => builds.find(x => x.m.source.repo === sel.repo && x.m.source.branch === sel.ref && x.m.chip === 'esp32s2');
async function loadFactoryStm() {
  try {
    const idx = await (await fetch('factory/index.json')).json();
    const get = async e => ({ m: await (await fetch('factory/' + e.manifest)).json(), base: 'factory/' + e.manifest.replace(/[^/]*$/, '') });
    factoryStm = await get(idx.builds.find(x => x.chip === 'stm32g070'));
    factoryV1 = await get(idx.builds.find(x => x.chip === 'esp32s2'));
  } catch (ex) { note(`factory: files not loaded: ${ex.message}`); }
}
const ERR_12 = 'Something went wrong. Unplug the USB cable. Hold the button while you plug it back in. Then press Try again.';
const ERR_34 = 'Something went wrong. Unplug the USB cable, wait 3 seconds, and plug it back in. Do not hold the button. Then press Try again.';
const NOT_RX = 'This is not the Rx board. Press Connect (step 1) or Reconnect (step 3) and choose the Rx board.';
const MID_REPLUG = 'Unplug the USB cable, wait 3 seconds, and plug it back in. Do not hold the button. The page continues by itself.';
const FS = [
  { t: 'Connect the Rx board', photo: true, a: ['Hold the button on the Rx board while you plug in its USB cable. Then let go.',
    'Press Connect. In the list, choose the Rx board (ESP32-S2), then press Connect in the list.'],
    b: 'Connect', done: 'Connected', pick: true, go: () => fsConnect('rom'), err: ERR_12 },
  { t: 'Restore, part 1 of 2', a: ['Keep the Rx board plugged in. This takes about 1 minute.'], b: 'Start', done: 'Part 1 done', go: fsHelper, err: ERR_12 },
  { t: 'Reconnect the Rx board', a: ['Press Reconnect. In the list, choose the Rx board (TinyUSB CDC), then press Connect in the list.'],
    noPick: 'Unplug the USB cable, wait 3 seconds, and plug it back in. Do not hold the button.',
    b: 'Reconnect', done: 'Connected', pick: true, go: () => fsConnect('app'), err: ERR_34 },
  { t: 'Restore, part 2 of 2', a: ['Keep the Rx board plugged in. This takes about 2 minutes.'], auto: true, go: fsRestore2, err: ERR_34,
    end: 'Finished. Your Rx board now has the factory firmware and factory settings.',
    endNote: "If the Rx board's USB drive does not appear, unplug the USB cable and plug it back in." },
];
let fst = { i: 0, err: null, busy: false, noPick: false }, fsPhase = 0;   // noPick: the list closed with no board picked
const isRxPort = port => { const i = port.getInfo(); return i.usbVendorId === ESPRESSIF && (i.usbProductId === ROM_PID || i.usbProductId === APP_PID); };
const fsBoard = test => [...boards.values()].find(x => x.state !== 'unplugged' && test(x));
async function fsConnect(mode) {
  let port;
  // the list shows only the Rx in the mode this step needs: other Espressif boards on the computer never appear
  // (a pick of another ESP32 board on the same hub once made the page read and reset it)
  try { port = guardPort(await navigator.serial.requestPort({ filters: [{ usbVendorId: ESPRESSIF, usbProductId: mode === 'rom' ? ROM_PID : APP_PID }] })); }
  catch { return null; }                               // list closed: no change
  note(`browser list: selected ${usbId(port)}`);
  if (!isRxPort(port)) throw Object.assign(plain(NOT_RX), { notRx: true });   // no command goes to it
  known(port); selPort = port;
  const b = boards.get(port);
  await connectBoard(b);
  // machine wait: the helper firmware's first start after the erase can answer late; read it again every 2 s, up to 20 s
  for (const t0 = Date.now(); mode === 'app' && b.mode === 'app' && !port.gone && Date.now() - t0 < 20000
    && (!b.connected || b.error || b.firmware !== 'rover firmware');) {
    note(`factory: no helper firmware answer yet (${b.error || b.firmware || 'no answer'}); read again`);
    await sleep(2000); await connectBoard(b);
  }
  if (!b.connected || b.error) throw plain('the Rx does not answer');
  if (mode === 'rom' && b.mode !== 'rom') throw plain('the Rx is not in update mode');
  if (mode === 'app' && b.mode === 'rom') throw plain('the Rx is in update mode');
  if (mode === 'app' && b.firmware !== 'rover firmware') throw plain('the Rx does not run the helper firmware');
  return mode === 'rom' ? 'update mode' : `helper firmware ${b.version || ''}`;
}
async function fsHelper() {
  const rover = factoryBuild(FACTORY_CFG.rover), b = fsBoard(x => x.mode === 'rom');
  if (!rover) throw plain('the helper firmware file did not load');
  if (!b) throw plain('the Rx is not in update mode');
  say(`factory: helper firmware ${rover.m.source.branch} ${rover.m.source.commit.slice(0, 7)}`);
  factoryHelper = true;
  try { await romLoad(b, rover, true); } finally { factoryHelper = false; }
  return 'helper firmware written';
}
async function fsStm() {
  const b = fsBoard(x => x.mode === 'app' && x.firmware === 'rover firmware');
  if (!b) throw plain('the Rx does not run the helper firmware');
  await ensureOpen(b);
  // the helper firmware learns the link version from the first good Rx STM32 frame (about 1.2 s after its start);
  // "stm v2" reports it: ver=1 is the 122 B frame of app 0.1.0, so no load and no replug (rover 0.7.15 bench.inc)
  const linkVer = async x => { const l = await ask(x, 'stm v2', /^STMV2 ver=/, 2000); const m = l && l.match(/ver=(\d+)/); return m ? +m[1] : null; };
  const v0 = await linkVer(b);
  say(`Rx STM32 link version: ${v0 == null ? 'no answer' : v0}`);
  if (v0 === 1 || /link v1/.test(b.stm || '')) { say('the Rx STM32 runs app 0.1.0 already: no load'); return; }
  await stmPart('', factoryStm);
  // machine waits: the ESP32 leaves the bridge 10 s after the last byte and restarts; the rover prints its link line
  // within about 3 s of its start (run J: 13 s and 3 s)
  const nb = await waitBoard(x => x.mode === 'app' && x.firmware === 'rover firmware' && x.state === 'open' && !x.bridge, 30000);
  if (!nb) throw plain('the Rx did not come back after the Rx STM32 load');
  let v1 = nb.lines.find(l => /STM link v1/.test(l)) || await waitLine(nb, /STM link v1/, 20000);
  if (!v1 && await linkVer(nb) === 1) v1 = 'STMV2 ver=1';
  if (!v1) throw plain('the Rx STM32 does not run firmware 0.1.0');
  say(`Rx STM32 runs the shipped app: ${v1}`);
}
async function fsV1() {
  const b = fsBoard(x => (x.mode === 'app' && x.firmware === 'rover firmware') || x.mode === 'rom');
  if (!b) throw plain('the Rx is not on USB');
  if (!factoryV1) throw plain('the V1 files did not load');
  if (b.mode === 'app') await ensureOpen(b);
  say(`factory: V1 ${factoryV1.m.source.commit.slice(0, 7)}`);
  factoryFinal = true;
  try { await romLoad(b, factoryV1, true); } finally { factoryFinal = false; }
}
async function fsRestore2() {                          // one step for the person: the Rx STM32 app 0.1.0, then V1
  fsPhase = 0; await fsStm();
  fsPhase = 1; if (job) job.pct = null;                // the Rx STM32's 100 % must not show as the step's 100 %
  await fsV1();
  return 'V1 written';
}
async function fsPress() {
  if (fst.busy) return;
  const k = fst.i, s = FS[k];
  if (!s || !s.go) return;
  fst.err = null; fst.busy = true; fst.started = true; render();
  let r = null;
  try {
    if (s.pick) r = await s.go();
    else {
      let err = null;
      await run(s.t, async () => { try { return await s.go(); } catch (ex) { err = ex; throw ex; } });
      if (err) throw err;
      r = job.result.text;
    }
    if (r) { fst.i = k + 1; fst.noPick = false; note(`factory step ${k + 1} done: ${r}`); }
    else if (s.pick && s.noPick) { fst.noPick = true; note(`factory step ${k + 1}: the list closed with no board picked`); }
  } catch (ex) { fst.err = ex.notRx ? NOT_RX : s.err; note(`factory step ${k + 1}: ${ex.message}`); }
  fst.busy = false; stepText = ''; render();
  if (r && FS[fst.i] && FS[fst.i].auto) fsPress();       // step 4 starts by itself after step 3
}
async function fsPickDuring() {                        // a pick the page asks for during a step (a device not yet allowed)
  let port;
  try { port = guardPort(await navigator.serial.requestPort({ filters: [{ usbVendorId: ESPRESSIF, usbProductId: hintRom ? ROM_PID : APP_PID }] })); } catch { return; }
  if (isRxPort(port)) known(port);
}
async function fsStartOver() {
  if (fst.busy) return;
  for (const x of [...boards.values()]) await disconnectBoard(x).catch(() => {});
  fst = { i: 0, err: null, busy: false, noPick: false }; fresh();
}
const factoryPicking = () => [ADD_HINT, PERM_HINT].some(h => stepText.includes(h));
function renderFactory(b, running) {
  const ok = !!(factoryBuild(FACTORY_CFG.rover) && factoryV1 && factoryStm);
  let h = '';
  FS.forEach((s, k) => {
    const st = k < fst.i ? 'done' : k === fst.i ? 'cur' : 'later';
    h += `<li class="fs ${st}"><span class="num">${st === 'done' ? '&#10003;' : k + 1}</span><div class="fb"><div class="ft">${esc(s.t)}`
      + (st === 'done' && s.done ? `<span class="fr">${esc(s.done)}</span>` : '') + '</div>';
    if (st === 'done' && s.end) h += `<div class="fgood">${esc(s.end)}</div><p>${esc(s.endNote)}</p>`;
    if (st === 'cur') {
      const person = running && !!stepText, writing = running && !stepText && !s.pick;
      if (s.photo) h += `<div class="photo fphoto"><img src="receiver.jpg" alt="Rx board">${$('arrows').outerHTML.replace(/ id="arrows"/, '').replace(/ hidden(="")?/, '')}</div>`;
      h += (person ? [MID_REPLUG] : fst.noPick && s.noPick ? [s.noPick, ...s.a] : s.a).map(l => `<p>${esc(l)}</p>`).join('');
      if (person && factoryPicking()) h += `<button id="fpick" class="primary">${esc(FS[2].b)}</button>`;
      if (writing) {
        let p = job && job.pct != null ? job.pct : null;
        if (p != null && s.auto) p = fsPhase === 0 ? Math.round(p * 0.3) : 30 + Math.round(p * 0.7);   // step 4: Rx STM32 30 %, V1 70 %
        h += `<div class="fbar${p == null ? ' ind' : ''}"><div style="width:${p == null ? 100 : p}%"></div></div><div class="fpct">${p == null ? '' : p + ' %'}</div>`;
      }
      if (fst.err && !fst.busy) h += `<div class="falert">${esc(fst.err)}</div><button id="fbtn" class="primary">${esc(fst.err === NOT_RX ? s.b : 'Try again')}</button>`;
      else if (s.b && !fst.busy && ok && serialOk) h += `<button id="fbtn" class="primary">${esc(s.b)}</button>`;
    }
    h += '</div></li>';
  });
  h = ('serial' in navigator ? '' : '<div class="fwarn">This page needs Chrome or Edge on a computer.</div>')
    + `<ol class="fsteps">${h}</ol>` + (fst.i > 0 && !fst.busy ? '<button id="freset" class="link">Start over</button>' : '');
  // settings only while the board can take a file: before the Connect press and after Finished
  $('cfgCard').classList.toggle('off', !(fst.i >= FS.length || (fst.i === 0 && !fst.started)));
  const box = $('fsteps');
  if (box.dataset.h !== h) {
    box.dataset.h = h; box.innerHTML = h;
    if ($('fbtn')) $('fbtn').onclick = () => fsPress();
    if ($('fpick')) $('fpick').onclick = () => fsPickDuring();
    if ($('freset')) $('freset').onclick = () => fsStartOver();
  }
}
window.factoryState = () => ({ i: fst.i, busy: fst.busy, err: fst.err, step: FS[fst.i] ? FS[fst.i].t : 'end', btn: $('fbtn') ? $('fbtn').textContent : '',
  pick: !!$('fpick'), person: !!stepText && !!(job && !job.done), stepText });

// ----- V1 settings editor: a text box with V1's config.json; the page checks it as V1 reads it
// (usb_msc_fat.cpp config_read_json at 172f102: C/C++ comments stripped outside strings, then a JSON parse; a key V1 does
// not know, or a value of the wrong type, V1 ignores with no message). "Save to Rx" writes the file through the
// browser's save dialog; the person saves it on the Rx drive. V1 reads a changed config.json 3 s after the last write.
const V1_KEYS = {                                      // key: the type V1 reads (int = whole number, u8 = whole number 0 to 255)
  vehicleID: 'u8', rx_channel: 'u8', CRUISE_THROTTLE: 'int', CREEP_THROTTLE: 'int', throttle_expo: 'num',
  PWM_OUT_1_DEFAULT: 'int', PWM_OUT_1_MIN: 'int', PWM_OUT_1_MAX: 'int', PWM_OUT_2_DEFAULT: 'int', PWM_OUT_2_MIN: 'int', PWM_OUT_2_MAX: 'int',
  PWM_OUT_3_DEFAULT: 'int', PWM_OUT_3_MIN: 'int', PWM_OUT_3_MAX: 'int', PWM_OUT_4_DEFAULT: 'int', PWM_OUT_4_MIN: 'int', PWM_OUT_4_MAX: 'int',
  kp: 'num', ki: 'num', kd: 'num', pid_direction: 'bool', enable_sensor_2: 'bool', rear_kp: 'num', rear_ki: 'num', rear_kd: 'num',
  rear_pid_direction: 'bool', reverse_rear_steer_in: 'bool', use_heading: 'bool', heading_weight: 'num', sensor_spacing_mm: 'num',
  stuck_threshold_mm: 'num', stuck_timeout_ms: 'num', BTxFollowChannel: 'u8', BTxThreshold: 'int', BTxFollowThreshold: 'int',
  BTxAccelRate: 'num', BTxCreepDuration: 'int', SineSwerveAmplitude: 'u8', SineSwerveFrequency: 'u8', CHARGE_VBATT_STOP: 'num',
  CHARGE_MAX_CURRENT: 'num', CHARGE_MAX_SECONDS: 'int', espnow_enabled: 'bool', tpa: 'num',
};
function v1Strip(t) {                                  // V1's comment strip; comments become spaces so positions stay the same
  let o = '', inS = false, esc1 = false, line = false, block = false;
  for (let i = 0; i < t.length; i++) {
    const c = t[i], n = t[i + 1];
    if (line) { if (c === '\n') { line = false; o += c; } else o += ' '; continue; }
    if (block) { if (c === '*' && n === '/') { block = false; o += '  '; i++; } else o += c === '\n' ? c : ' '; continue; }
    if (inS) { o += c; if (esc1) esc1 = false; else if (c === '\\') esc1 = true; else if (c === '"') inS = false; continue; }
    if (c === '"') { inS = true; o += c; continue; }
    if (c === '/' && n === '/') { line = true; o += '  '; i++; continue; }
    if (c === '/' && n === '*') { block = true; o += '  '; i++; continue; }
    o += c;
  }
  return o;
}
const lineOf = (t, pos) => t.slice(0, pos).split('\n').length;
function v1Check(text) {
  const s = v1Strip(text), errs = [], warns = [];
  let doc;
  try { doc = JSON.parse(s); }
  catch (ex) {
    const m = ex.message.match(/position (\d+)/), pos = m ? +m[1] : s.length, what = ex.message, ln = lineOf(s, pos);
    const before = s.slice(0, pos), endAt = before.search(/\S\s*$/), endLn = endAt < 0 ? 1 : lineOf(s, endAt);
    if (/Expected ',' or '[}\]]' after/.test(what)) errs.push(endLn < ln ? `Line ${ln}: a comma is missing at the end of line ${endLn}.` : `Line ${ln}: this value is not valid.`);
    else if (/property name/.test(what) && /,\s*$/.test(before)) errs.push(`Line ${endLn}: a comma too many.`);
    else if (/property name/.test(what)) errs.push(`Line ${ln}: a setting name must be in double quotes.`);
    else if (/Unexpected end/.test(what)) errs.push(`Line ${ln}: the file ends too early.`);
    else errs.push(`Line ${ln}: this line is not valid.`);
    return { errs, warns };
  }
  if (!doc || typeof doc !== 'object' || Array.isArray(doc)) { errs.push('Line 1: the settings must be one { } block.'); return { errs, warns }; }
  for (const [k, v] of Object.entries(doc)) {
    const ty = V1_KEYS[k];
    if (!ty) warns.push(`Unknown setting "${k}". The Rx board ignores it.`);
    else if (ty === 'bool' && typeof v !== 'boolean') warns.push(`${k} must be true or false.`);
    else if ((ty === 'int' || ty === 'u8') && !Number.isInteger(v)) warns.push(`${k} must be a whole number.`);
    else if (ty === 'num' && typeof v !== 'number') warns.push(`${k} must be a number.`);
    else if (ty === 'u8' && (v < 0 || v > 255)) warns.push(`${k} must be 0 to 255.`);
  }
  return { errs, warns };
}
let v1Default = '';
async function v1EditorLoad() {
  try { v1Default = await (await fetch('factory/v1_config.json')).text(); } catch { v1Default = ''; }
  if (!$('cfgText').value) $('cfgText').value = v1Default;
  v1EditorCheck();
}
function v1EditorCheck() {
  const { errs, warns } = v1Check($('cfgText').value);
  const rows = [...errs.map(t => `<div class="ce">${esc(t)}</div>`), ...warns.map(t => `<div class="cw">${esc(t)}</div>`)];
  $('cfgCheck').innerHTML = rows.join('');
  $('cfgSave').disabled = errs.length > 0;
  // the line to fix: the last line the error names ("Line 4: a comma is missing at the end of line 3." marks line 3)
  const m = errs.length ? [...errs[0].matchAll(/line (\d+)/gi)].pop() : null;
  cfgErrLine = m ? +m[1] : 0;
  const n = $('cfgText').value.split('\n').length, g = Array.from({ length: n }, (_, i) => i + 1).join('\n');
  if ($('cfgGut').textContent !== g) $('cfgGut').textContent = g;
  v1EditorScroll();
}
let cfgErrLine = 0;
function v1EditorScroll() {                            // the line numbers and the error line follow the text when it scrolls
  const ta = $('cfgText'), hi = $('cfgHi');
  $('cfgGut').scrollTop = ta.scrollTop;
  hi.hidden = !cfgErrLine;
  if (cfgErrLine) hi.style.top = `${8 + (cfgErrLine - 1) * 19 - ta.scrollTop}px`;
}
async function v1EditorSave() {
  const text = $('cfgText').value;
  if (v1Check(text).errs.length) return;
  try {
    if (window.showSaveFilePicker) {
      const h = await showSaveFilePicker({ suggestedName: 'config.json', types: [{ description: 'JSON', accept: { 'application/json': ['.json'] } }] });
      const w = await h.createWritable(); await w.write(text); await w.close();
    } else {
      const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type: 'application/json' })); a.download = 'config.json'; a.click();
    }
    note('settings: config.json saved');
  } catch (ex) { if (ex.name !== 'AbortError') note(`settings: not saved: ${ex.message}`); }
}
function v1EditorStart() {
  $('cfgText').addEventListener('input', () => v1EditorCheck());
  $('cfgText').addEventListener('scroll', () => v1EditorScroll());
  $('cfgSave').onclick = () => v1EditorSave();
  $('cfgReset').onclick = () => { $('cfgText').value = v1Default; v1EditorCheck(); };
  v1EditorLoad();
}

async function fetchPool(urls, n) {
  const out = new Array(urls.length).fill(null);
  let next = 0;
  const worker = async () => {
    while (next < urls.length) {
      const k = next++;
      for (let t = 0; t < 3 && !out[k]; t++) { try { const r = await fetch(urls[k]); if (r.ok) out[k] = await r.json(); } catch { await sleep(500 * (t + 1)); } }
    }
  };
  await Promise.all(Array.from({ length: Math.min(n, urls.length) }, worker));
  return out;
}
async function start() {
  $('refBtn').onclick = e => { e.stopPropagation(); $('refList').hidden = !$('refList').hidden; };
  addEventListener('click', e => { if (!$('fwCard').contains(e.target)) $('refList').hidden = true; });
  $('rxBtn').onclick = $('rxBtn2').onclick = () => { const b = selected(); fresh(); if (b) disconnectBoard(b); };
  $('doneBtn').onclick = () => { ready1 = true; render(); $('findBtn').focus(); };
  addEventListener('keydown', e => {
    if (ready1 || (e.key !== 'Enter' && e.key !== ' ') || e.target.closest('button, input, select, a, summary')) return;
    e.preventDefault(); ready1 = true; render(); $('findBtn').focus();
  });
  $('findBtn').onclick = async () => {
    try {
      const port = guardPort(await navigator.serial.requestPort({ filters: [{ usbVendorId: ESPRESSIF }] }));
      note(`browser list: selected ${usbId(port)}`);
      known(port);
      if (!job || job.done) { selPort = port; connectBoard(boards.get(port)); }
    } catch { /* the browser list was closed */ }
  };
  $('updBtn').onclick = () => startUpdate();
  if (FACTORY) { document.body.classList.add('factory'); loadFactoryStm().then(render); v1EditorStart(); }
  $('eraseBtn').onclick = () => startErase();
  $('more').ontoggle = () => render();
  try {
    const v = await (await fetch('version.json', { cache: 'no-store' })).json();
    if (v && v.version) pageVer = `Page v${v.version}, ${v.date}, ${v.commit}`;
  } catch { /* no version.json: a local copy */ }
  $('pageVer').textContent = pageVer;
  if (window.PROGRAM_TEST) { buildsLoaded = true; return render(); }
  if (!('serial' in navigator) || !isSecureContext) {
    serialOk = false;
    serialMsg = !isSecureContext
      ? `This page needs a secure address for USB. Open http://localhost${location.port ? ':' + location.port : ''}${location.pathname} on the computer with the Rx.`
      : 'No USB in this browser. To install, use Chrome or Edge on a computer.';
  }
  render();
  if (serialOk) {
    navigator.serial.addEventListener('connect', e => { const port = e.target; port.gone = false; lastConnect = Date.now(); note(`USB connect: ${usbId(port)}`); setTimeout(() => { if (!port.gone && (!FACTORY || isRxPort(port))) known(port); }, 2000); });
    navigator.serial.addEventListener('disconnect', e => {
      const port = e.target, b = boards.get(port);
      note(`USB disconnect: ${usbId(port)}`);
      port.gone = true;
      disconnects++;
      forceClose(port).catch(() => {});
      if (b) { b.state = 'unplugged'; closeBoard(b).catch(() => {}); }
      if (port === selPort && !(job && !job.done)) fresh();
    });
    addEventListener('pagehide', () => { for (const b of boards.values()) b.closing = true; releaseAll(); });
    setInterval(() => {
      if ((job && !job.done) || [...boards.values()].some(x => x.detecting)) return;
      const link = [...boards.values()].find(x => x.rom && x.connected);
      const keep = link ? link.port : null;
      if ([...openPorts].some(p => p !== keep)) { note('sweep: a port was left open; closed'); releaseAll(keep).then(render); }
    }, 2000);
    const permitted = await navigator.serial.getPorts();
    note(`the browser permits ${permitted.length} port${permitted.length === 1 ? '' : 's'} on USB now${permitted.length ? ': ' + permitted.map(usbId).join(', ') : ''}; modes allowed before in this browser: ${['app', 'rom'].filter(m => perm[m]).map(m => m === 'rom' ? 'update mode' : 'firmware runs').join(', ') || 'none'}`);
    for (const p of permitted) if (!FACTORY || isRxPort(p)) known(p);   // factory: another board allowed earlier is never opened
  }
  try {
    let r = await fetch('sources.json').catch(() => null);
    if (!r || !r.ok) r = await fetch('api/sources');
    for (const s of (await r.json()).sources) {
      const idx = await (await fetch(s.base + 'index.json')).json();
      const rr = await fetch(s.base + 'releases.json').catch(() => null), rel = rr && rr.ok ? (await rr.json()).commits || {} : null;
      const rf = await fetch(s.base + 'refs.json').catch(() => null);
      if (rf && rf.ok) for (const e of (await rf.json()).refs || []) refInfo[e.repo + ' ' + e.ref] = e;
      const ms = await fetchPool(idx.builds.map(e => s.base + e.manifest), 6);
      idx.builds.forEach((e, k) => { if (ms[k]) builds.push({ m: ms[k], base: s.base + e.manifest.replace(/[^/]*$/, ''), src: s.name, release: rel ? rel[ms[k].source.commit] || false : undefined }); });
      const lost = ms.filter(m => !m).length; if (lost) note(`${lost} of ${ms.length} build manifests did not load`);
    }
    builds.sort((a, b) => b.m.source.built_utc.localeCompare(a.m.source.built_utc));
    allBuilds = [...builds];
    const seen = new Set();
    builds = builds.filter(x => { const k = [x.m.source.repo, x.m.source.branch, x.m.chip].join(' '); return !seen.has(k) && seen.add(k); });
  } catch (ex) { console.error(ex); }
  buildsLoaded = true;
  render();
}
start();
