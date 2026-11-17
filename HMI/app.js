/* ==========================================================================
   HMI Banco de Pruebas · BP_A_Master_F
   --------------------------------------------------------------------------
   Arquitectura:
     STM32 (RAM) ──SWD── ST-Link ── OpenOCD ──TCL:6666── Puente Python ──WS── HMI
   La HMI no habla con el hardware: solo intercambia JSON con el puente por
   WebSocket (ver README.md para el protocolo). Incluye un simulador para
   probar la interfaz sin hardware.
   ========================================================================== */
'use strict';

/* ------------------------------------------------------------------------ */
/* Configuración y metadatos del firmware                                    */
/* ------------------------------------------------------------------------ */
const TICKS_PER_GEAR_REV = 2520;   // 60 ticks/vuelta motor x 42 dientes (sinfin de 1 entrada)

const DEFAULTS = {
  source: 'demo',               // 'bridge' | 'demo'
  wsUrl: 'ws://localhost:8765',
  ppr: 360,                     // 360° por vuelta del engranaje (2520 pulsos)
  chartWindow: 30,              // segundos
  canTimeoutMs: 1000,
};

const settings = Object.assign({}, DEFAULTS, loadJSON('hmi.settings', {}));

/* Espejo de main.c (BP_A_Master_F y BP_B_Salve_F) */
const MOTOR_STATES = {
  0: { label: 'IDLE',        desc: 'Detenido fuera de Home', cls: 'st-idle',   color: '#8b97a8' },
  1: { label: 'SCANNING',    desc: 'Barrido oscilante',      cls: 'st-scan',   color: '#3ea6ff' },
  2: { label: 'HOMED',       desc: 'En posición Home (±8°)', cls: 'st-homed',  color: '#2ecc71' },
  3: { label: 'HOMING',      desc: 'Regresando a Home',      cls: 'st-homing', color: '#f5b942' },
  4: { label: 'ERROR',       desc: 'Falla reportada por B',  cls: 'st-error',  color: '#ff4d4f' },
  5: { label: 'POSITIONING', desc: 'Posicionando ángulo',    cls: 'st-pos',    color: '#00d2d3' },
};

const MASTER_STATES = {
  0: { label: 'INIT',        desc: 'Inicializando',                 cls: '' },
  1: { label: 'LOCAL',       desc: 'Operación local (BTN1..4)',     cls: 'mst-local' },
  2: { label: 'REMOTE',      desc: 'Operación remota (PC / HMI)',   cls: 'mst-remote' },
  3: { label: 'CAN_FAULT',   desc: 'Timeout de enlace CAN (>500ms)', cls: 'mst-fault' },
  4: { label: 'SLAVE_ERROR', desc: 'Falla activa en Placa B',       cls: 'mst-err' },
};

const COMMANDS = {
  1:  'Barrido',
  2:  'Fijar Home',
  3:  'Ir a Home',
  4:  'STOP',
  5:  'Giro +30° (+210 ticks)',
  6:  'Giro +90° (+630 ticks)',
  7:  'Giro +180° (+1260 ticks)',
  8:  'Giro +360° (+2520 ticks)',
  9:  'Giro −30° (−210 ticks)',
  10: 'Giro −90° (−630 ticks)',
  11: 'Giro −180° (−1260 ticks)',
  12: 'Giro −360° (−2520 ticks)',
};

const STEP_CMD_MAP = {
  1:  { 30: 5, 90: 6, 180: 7, 360: 8 },
  '-1': { 30: 9, 90: 10, 180: 11, 360: 12 },
};

const MODE = { LOCAL: 0, REMOTE: 1 };

/* Metadatos por defecto; el puente puede sobrescribirlos (addr, type, etc.) */
const DEFAULT_VAR_META = {
  live_position:       { type: 'int32_t',         writable: false, desc: 'Posición real del engranaje en grados (360° = 1 vuelta)' },
  live_position_ticks: { type: 'int32_t',         writable: false, desc: 'Posición cruda en pulsos de encoder (2520 ticks = 360°)' },
  live_motor_state:    { type: 'uint8_t',         writable: false, desc: '0=IDLE 1=SCANNING 2=HOMED 3=HOMING 4=ERROR 5=POSITIONING' },
  master_state:        { type: 'MasterState_t',   writable: false, desc: 'FSM Master: 0=INIT 1=LOCAL 2=REMOTE 3=CAN_FAULT 4=SLAVE_ERROR' },
  telemetry_counter:   { type: 'uint32_t',        writable: false, desc: 'Paquetes de telemetría CAN 0x030 recibidos' },
  system_mode:         { type: 'OperatingMode_t', writable: true,  desc: '0=Local, 1=Remoto' },
  remote_cmd:          { type: 'uint8_t',         writable: true,  desc: '1..4=Botones, 5..12=Giros por ángulo (se autolimpia)' },
  last_system_mode:    { type: 'OperatingMode_t', writable: false, desc: 'Detector de cambio de modo' },
  current_led_state:   { type: 'uint8_t',         writable: false, desc: 'Reflejo del LED PB0 de la placa B' },
};

const WRITE_TIMEOUT_MS = 2000;
const CMD_TIMEOUT_MS = 3000;
const STALE_MS = 2000;
const HISTORY_MAX_MS = 125 * 1000;
const HOMED_TOLERANCE_DEG = 8;  // Tolerancia de ±8° alrededor de 0° (contempla deslizamiento mecánico por inercia)

/* ------------------------------------------------------------------------ */
/* Estado de la aplicación                                                   */
/* ------------------------------------------------------------------------ */
const app = {
  transport: null,
  link: 'disconnected',         // disconnected | connecting | connected
  target: 'unknown',            // running | halted | unknown
  haltStartT: 0,
  vars: {},
  varFlashT: {},
  meta: structuredClone(DEFAULT_VAR_META),
  info: {},                     // datos del mensaje status (elf, rate, etc.)

  history: [],                  // { t, pos, st }
  counterSamples: [],           // { t, c }
  canRate: null,
  lastCounter: null,
  lastCounterChangeT: 0,
  lastMsgT: 0,

  pendingWrites: new Map(),     // id -> { name, value, t }
  nextId: 1,
  pendingCmd: null,             // { n, t, acked }
  lastCmdResult: null,          // { text, cls, t }
  stepDir: 1,                   // 1 = sentido positivo (+), -1 = sentido negativo (-)

  recording: false,
  csvRows: [],
  chartPaused: false,
  chartFreezeT: 0,
  renderQueued: false,
};

/* ------------------------------------------------------------------------ */
/* Utilidades                                                                */
/* ------------------------------------------------------------------------ */
const $ = (id) => document.getElementById(id);

function loadJSON(key, fallback) {
  try { return JSON.parse(localStorage.getItem(key)) ?? fallback; } catch { return fallback; }
}
function saveSettings() { localStorage.setItem('hmi.settings', JSON.stringify(settings)); }

function fmtInt(n) {
  if (n === undefined || n === null || Number.isNaN(n)) return '—';
  return Number(n).toLocaleString('es-AR');
}
function fmtSigned(n) {
  if (n === undefined || n === null) return '—';
  n = n + 0;                                      // normaliza -0 a 0
  return (n > 0 ? '+' : '') + fmtInt(n);
}
function fmtHex(n) {
  if (n === undefined || n === null) return '—';
  return '0x' + Number(n).toString(16).toUpperCase().padStart(8, '0');
}
function timeStamp() {
  const d = new Date();
  return d.toLocaleTimeString('es-AR', { hour12: false }) + '.' + String(d.getMilliseconds()).padStart(3, '0');
}
function stateInfo(st) {
  return MOTOR_STATES[st] || { label: st === undefined ? '—' : `? (${st})`, desc: 'Estado desconocido', cls: '', color: '#888' };
}
function masterStateInfo(mst) {
  return MASTER_STATES[mst] || { label: mst === undefined ? '—' : `? (${mst})`, desc: 'Estado desconocido', cls: '' };
}
function effectiveMotorState(st, pos) {
  if (st === 0 && pos !== undefined && Math.abs(pos) <= HOMED_TOLERANCE_DEG) return 2;
  return st;
}
function modeName(m) { return m === MODE.REMOTE ? 'REMOTO' : m === MODE.LOCAL ? 'LOCAL' : `? (${m})`; }

/* ------------------------------------------------------------------------ */
/* Registro de eventos                                                       */
/* ------------------------------------------------------------------------ */
function log(msg, level = 'info') {
  const list = $('log-list');
  const li = document.createElement('li');
  li.className = level;
  li.innerHTML = `<span class="ts">${timeStamp()}</span><span></span>`;
  li.lastChild.textContent = msg;
  list.prepend(li);
  while (list.children.length > 300) list.lastChild.remove();
}

/* ------------------------------------------------------------------------ */
/* Transporte: puente Python por WebSocket                                   */
/* ------------------------------------------------------------------------ */
class BridgeTransport {
  constructor(url, onMessage, onLink) {
    this.url = url;
    this.onMessage = onMessage;
    this.onLink = onLink;
    this.ws = null;
    this.retryTimer = null;
    this.closedByUser = false;
    this.everConnected = false;
  }

  connect() {
    this.closedByUser = false;
    this._open();
  }

  _open() {
    this.onLink('connecting');
    try {
      this.ws = new WebSocket(this.url);
    } catch (e) {
      log(`URL inválida: ${e.message}`, 'err');
      this.onLink('disconnected');
      return;
    }
    this.ws.onopen = () => {
      this.everConnected = true;
      this.onLink('connected');
      this.send({ type: 'hello', client: 'hmi', version: 1 });
    };
    this.ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      this.onMessage(msg);
    };
    this.ws.onclose = () => {
      this.ws = null;
      if (this.closedByUser) { this.onLink('disconnected'); return; }
      this.onLink('connecting');               // reintento automático
      this.retryTimer = setTimeout(() => this._open(), 2000);
    };
    this.ws.onerror = () => { /* onclose se encarga */ };
  }

  send(msg) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(msg));
      return true;
    }
    return false;
  }

  close() {
    this.closedByUser = true;
    clearTimeout(this.retryTimer);
    if (this.ws) this.ws.close(); else this.onLink('disconnected');
  }
}

/* ------------------------------------------------------------------------ */
/* Transporte: simulador (firmware A + placa B + debugger)                   */
/* ------------------------------------------------------------------------ */
class DemoTransport {
  constructor(onMessage, onLink) {
    this.onMessage = onMessage;
    this.onLink = onLink;
    this.timer = null;
    this.t0 = performance.now();
    this.tick_n = 0;
    this.halted = false;
    this.canFault = false;

    /* Variables de la placa A (como en main.c) */
    this.fw = {
      live_position: 0, live_motor_state: 2, master_state: 1, telemetry_counter: 0,
      system_mode: 0, remote_cmd: 0, last_system_mode: 0, current_led_state: 0,
    };
    /* Modelo de la placa B (en grados del engranaje: 360° = 1 vuelta) */
    this.PPR = 360;
    this.SPEED = 180;                           // grados/s
    this.slave = { rawPos: 0, pos: 0, state: 2, dir: 1, targetPos: 0 };
  }

  connect() {
    this.onLink('connecting');
    setTimeout(() => {
      this.onLink('connected');
      const meta = {};
      let addr = 0x20000010;
      for (const [k, m] of Object.entries(DEFAULT_VAR_META)) {
        meta[k] = { ...m, addr };
        addr += 4;
      }
      this.onMessage({
        type: 'status', openocd: true, target: 'running',
        elf: 'BP_A_Master_F.elf (simulado)', rate_hz: 16, vars_meta: meta,
      });
      this.timer = setInterval(() => this._tick(), 20);
    }, 300);
  }

  close() {
    clearInterval(this.timer);
    this.timer = null;
    this.onLink('disconnected');
  }

  send(msg) {
    if (!this.timer) return false;
    if (msg.type === 'write') {
      const ok = msg.var in this.fw && Number.isFinite(Number(msg.value));
      if (ok) this.fw[msg.var] = Number(msg.value);   // la RAM se escribe aunque el micro esté en halt
      setTimeout(() => this.onMessage({
        type: 'ack', id: msg.id, ok, error: ok ? undefined : `Variable desconocida o valor inválido: ${msg.var}`,
      }), 8);
    }
    return true;
  }

  /* --- Acciones del panel de simulación --- */
  setHalted(h) {
    this.halted = h;
    this.onMessage({ type: 'target', state: h ? 'halted' : 'running' });
  }
  setCanFault(f) { this.canFault = f; }
  injectError() { this.slave.state = 4; }
  physicalButton(n) {
    if (this.halted) return;
    const fw = this.fw;
    if (n === 5) { fw.system_mode = 0; fw.last_system_mode = 0; fw.current_led_state = 0; }
    if (n === 6) { fw.system_mode = 1; fw.last_system_mode = 1; fw.current_led_state = 1; }
    if (n === 4) this._canSend(0x010, 4);
  }

  /* --- Simulación --- */
  _canSend(id, data) {
    if (this.canFault) return false;
    if (id === 0x010) this._slaveCommand(data);
    return true;
  }

  _slaveCommand(cmd) {
    const s = this.slave;
    if (cmd === 4) { s.state = Math.abs(s.pos) <= HOMED_TOLERANCE_DEG ? 2 : 0; s.targetPos = s.pos; return; }
    if (s.state === 4) {
      if (cmd === 2) { s.rawPos = 0; s.pos = 0; s.targetPos = 0; s.state = 2; }
      if (cmd === 1) { s.state = Math.abs(s.pos) <= HOMED_TOLERANCE_DEG ? 2 : 0; }
      return;
    }
    if (cmd === 1) { s.state = 1; s.dir = s.pos >= 0 ? 1 : -1; }
    if (cmd === 2) { s.rawPos = 0; s.pos = 0; s.targetPos = 0; s.state = 2; }
    if (cmd === 3) { s.targetPos = 0; s.state = Math.abs(s.pos) <= 1 ? 2 : 3; }
    const stepDegMap = { 5: 30, 6: 90, 7: 180, 8: 360, 9: -30, 10: -90, 11: -180, 12: -360 };
    if (cmd in stepDegMap) {
      s.targetPos = ((s.state === 3 || s.state === 5) && s.targetPos !== undefined ? s.targetPos : s.pos) + stepDegMap[cmd];
      s.state = 5;
    }
  }

  _slavePhysics(dt) {
    const s = this.slave;
    const step = this.SPEED * dt;
    const amp = 2 * this.PPR;                     // barrido de ±2 vueltas
    if (s.state === 1) {
      s.rawPos += s.dir * step;
      if (s.rawPos >= amp) { s.rawPos = amp; s.dir = -1; }
      if (s.rawPos <= -amp) { s.rawPos = -amp; s.dir = 1; }
    } else if (s.state === 3 || s.state === 5) {
      const tgt = s.targetPos ?? 0;
      const err = tgt - s.rawPos;
      if (Math.abs(err) <= step) {
        s.rawPos = tgt;
        s.state = Math.abs(s.rawPos) <= HOMED_TOLERANCE_DEG ? 2 : 0;
      } else {
        s.rawPos += Math.sign(err) * step;
      }
    }
    s.pos = Math.round(s.rawPos) + 0;
  }

  _firmwareLoop() {
    const fw = this.fw;
    if (fw.system_mode !== fw.last_system_mode) {
      if (fw.system_mode === 1) { fw.current_led_state = 1; this._canSend(0x020, 1); }
      else { fw.system_mode = 0; fw.current_led_state = 0; this._canSend(0x020, 0); }
      fw.last_system_mode = fw.system_mode;
    }
    if (this.canFault) {
      fw.master_state = 3;                        // MASTER_CAN_FAULT
      fw.remote_cmd = 0;
      return;
    }
    if (fw.live_motor_state === 4) {
      fw.master_state = 4;                        // MASTER_SLAVE_ERROR
    } else {
      fw.master_state = fw.system_mode === 1 ? 2 : 1;
    }
    if (fw.system_mode === 0) {
      fw.remote_cmd = 0;                          // en local se descartan comandos remotos
    } else if (fw.remote_cmd !== 0) {
      if (fw.remote_cmd >= 1 && fw.remote_cmd <= 12) this._canSend(0x010, fw.remote_cmd);
      fw.remote_cmd = 0;
    }
  }

  _tick() {
    this.tick_n++;
    const dt = 0.02;
    if (!this.halted) {
      this._slavePhysics(dt);
      this._firmwareLoop();
      /* Placa B envía telemetría 0x030 a 50 Hz (cada 20 ms) */
      if (!this.canFault) {
        this.fw.live_position = this.slave.pos;
        this.fw.live_position_ticks = Math.round((this.slave.pos * TICKS_PER_GEAR_REV) / 360);
        this.fw.live_motor_state = this.slave.state;
        this.fw.telemetry_counter = (this.fw.telemetry_counter + 1) >>> 0;
      }
    }
    /* El puente sondea la RAM cada 60 ms (~16 Hz) */
    if (this.tick_n % 3 !== 0) return;
    this.onMessage({
      type: 'telemetry',
      t: Math.round(performance.now() - this.t0),
      target: this.halted ? 'halted' : 'running',
      vars: { ...this.fw },
    });
  }
}

/* ------------------------------------------------------------------------ */
/* Manejo de mensajes del puente                                             */
/* ------------------------------------------------------------------------ */
function onMessage(msg) {
  switch (msg.type) {
    case 'telemetry': onTelemetry(msg); break;
    case 'status':    onStatus(msg); break;
    case 'target':    setTarget(msg.state); break;
    case 'ack':       onAck(msg); break;
    case 'log':       log(`[puente] ${msg.message}`, msg.level || 'info'); break;
    case 'error':     log(`[puente] ${msg.message}`, 'err'); break;
    default: break;
  }
  scheduleRender();
}

function onStatus(msg) {
  app.info = { ...app.info, ...msg };
  if (msg.vars_meta) {
    for (const [k, m] of Object.entries(msg.vars_meta)) {
      app.meta[k] = { ...(app.meta[k] || {}), ...m };
    }
    buildVarsTable();
  }
  if (msg.target) setTarget(msg.target);
  const parts = [];
  if (msg.elf) parts.push(`ELF: ${msg.elf}`);
  if (msg.rate_hz) parts.push(`muestreo ${msg.rate_hz} Hz`);
  if ('openocd' in msg) parts.push(msg.openocd ? 'OpenOCD OK' : 'OpenOCD NO conectado');
  if (parts.length) log(`Estado del puente: ${parts.join(' · ')}`, msg.openocd === false ? 'err' : 'ok');
}

function setTarget(state) {
  if (!state || state === app.target) return;
  const prev = app.target;
  app.target = state;
  const now = performance.now();
  if (state === 'halted') {
    app.haltStartT = now;
    log('Micro DETENIDO (breakpoint / pausa). Valores congelados.', 'halt');
  } else if (state === 'running') {
    app.lastCounterChangeT = now;                 // no disparar la alarma CAN apenas reanuda
    if (prev === 'halted') log('Micro REANUDADO.', 'ok');
  } else if (state === 'reset') {
    log('Micro en reset.', 'warn');
  }
}

function onTelemetry(msg) {
  const now = performance.now();
  app.lastMsgT = now;
  if (msg.target) setTarget(msg.target);
  const v = msg.vars || {};

  /* Detección de cambios */
  for (const [k, val] of Object.entries(v)) {
    const prev = app.vars[k];
    if (prev !== val) {
      app.varFlashT[k] = now;
      if (prev !== undefined) onVarChange(k, prev, val);
    }
  }
  Object.assign(app.vars, v);
  if (Object.keys(v).some((k) => !(k in app.meta))) {
    for (const k of Object.keys(v)) if (!(k in app.meta)) app.meta[k] = { type: '?', writable: false, desc: '' };
    buildVarsTable();
  }

  /* Tasa de telemetría CAN a partir de telemetry_counter */
  if ('telemetry_counter' in v) {
    const c = v.telemetry_counter;
    if (app.lastCounter === null || c !== app.lastCounter) app.lastCounterChangeT = now;
    app.lastCounter = c;
    app.counterSamples.push({ t: now, c });
    while (app.counterSamples.length > 2 && now - app.counterSamples[0].t > 2000) app.counterSamples.shift();
    const first = app.counterSamples[0];
    const dt = (now - first.t) / 1000;
    app.canRate = dt > 0.3 ? ((c - first.c) >>> 0) / dt : null;
  }

  /* Historial para el gráfico (no se agregan puntos mientras está en halt) */
  if (app.target !== 'halted' && 'live_position' in v) {
    const effSt = effectiveMotorState(v.live_motor_state, v.live_position);
    app.history.push({ t: now, pos: v.live_position, st: effSt });
    while (app.history.length && now - app.history[0].t > HISTORY_MAX_MS) app.history.shift();
  }

  checkPendingCmd(now);

  if (app.recording) {
    app.csvRows.push([
      new Date().toISOString(), msg.t ?? '', app.target,
      v.live_position, v.live_position_ticks ?? '', v.live_motor_state, v.master_state ?? '', v.telemetry_counter, v.system_mode, v.remote_cmd,
    ]);
  }
}

function onVarChange(name, prev, val) {
  if (name === 'live_motor_state') {
    const a = stateInfo(prev), b = stateInfo(val);
    log(`FSM Motor (B): ${a.label} → ${b.label}`, val === 4 ? 'err' : 'info');
  } else if (name === 'master_state') {
    const a = masterStateInfo(prev), b = masterStateInfo(val);
    const lvl = val === 3 || val === 4 ? 'err' : 'info';
    log(`FSM Master (A): ${a.label} → ${b.label}`, lvl);
  } else if (name === 'system_mode') {
    log(`Modo: ${modeName(prev)} → ${modeName(val)}`, 'info');
  }
}

/* ------------------------------------------------------------------------ */
/* Escrituras y comandos                                                     */
/* ------------------------------------------------------------------------ */
function writeVar(name, value) {
  if (!app.transport || app.link !== 'connected') {
    log('No hay conexión con el puente.', 'err');
    return false;
  }
  const id = app.nextId++;
  const ok = app.transport.send({ type: 'write', id, var: name, value: Number(value) });
  if (!ok) { log(`No se pudo enviar la escritura de ${name}.`, 'err'); return false; }
  app.pendingWrites.set(id, { name, value: Number(value), t: performance.now() });
  log(`→ ${name} = ${value}`, 'info');
  return true;
}

function onAck(msg) {
  const pw = app.pendingWrites.get(msg.id);
  if (!pw) return;
  app.pendingWrites.delete(msg.id);
  if (msg.ok) {
    if (pw.name === 'remote_cmd' && app.pendingCmd && app.pendingCmd.n === pw.value) app.pendingCmd.acked = true;
  } else {
    log(`✗ Escritura ${pw.name} = ${pw.value} rechazada: ${msg.error || 'error desconocido'}`, 'err');
    if (pw.name === 'remote_cmd') app.pendingCmd = null;
  }
}

function setMode(mode) {
  if (app.vars.system_mode === mode) return;
  writeVar('system_mode', mode);
}

function sendCommand(n) {
  if (app.link !== 'connected') { log('No hay conexión con el puente.', 'err'); return; }
  if (app.vars.system_mode !== MODE.REMOTE) {
    hint('El sistema está en modo LOCAL: el firmware descarta remote_cmd. Pasá a REMOTO primero.');
    log(`Comando "${COMMANDS[n]}" no enviado: modo LOCAL.`, 'warn');
    return;
  }
  if (app.pendingCmd && n !== 4) {
    hint(`Esperando que el firmware tome "${COMMANDS[app.pendingCmd.n]}".`);
    return;
  }
  if (writeVar('remote_cmd', n)) {
    app.pendingCmd = { n, t: performance.now(), acked: false };
    app.lastCmdResult = null;
  }
  scheduleRender();
}

function checkPendingCmd(now) {
  const pc = app.pendingCmd;
  if (!pc) return;
  if (app.target === 'halted') { pc.t = now; return; }   // el reloj no corre mientras está en pausa
  if (pc.acked && app.vars.remote_cmd === 0) {
    const ms = Math.round(now - pc.t);
    log(`✓ "${COMMANDS[pc.n]}" tomado por el firmware (~${ms} ms)`, 'ok');
    app.lastCmdResult = { text: `✓ ${COMMANDS[pc.n]} ejecutado`, cls: 'done', t: now };
    app.pendingCmd = null;
  } else if (now - pc.t > CMD_TIMEOUT_MS) {
    log(`✗ "${COMMANDS[pc.n]}" no fue procesado por el firmware (timeout).`, 'err');
    app.lastCmdResult = { text: `✗ ${COMMANDS[pc.n]}: timeout`, cls: 'pending', t: now };
    app.pendingCmd = null;
  }
}

let hintTimer = null;
function hint(text) {
  $('control-hint').textContent = text;
  clearTimeout(hintTimer);
  hintTimer = setTimeout(() => { $('control-hint').textContent = ''; }, 4000);
}

/* ------------------------------------------------------------------------ */
/* Conexión                                                                  */
/* ------------------------------------------------------------------------ */
function onLink(state) {
  const prev = app.link;
  app.link = state;
  if (state === 'connected' && prev !== 'connected') {
    log(settings.source === 'demo' ? 'Simulador iniciado.' : `Conectado al puente ${settings.wsUrl}`, 'ok');
    app.lastMsgT = performance.now();
    app.lastCounterChangeT = performance.now();
  }
  if (state !== 'connected' && prev === 'connected') {
    log('Conexión perdida con el puente.', 'warn');
    resetRuntime();
  }
  if (state === 'disconnected') resetRuntime();
  scheduleRender();
}

function resetRuntime() {
  app.target = 'unknown';
  app.pendingWrites.clear();
  app.pendingCmd = null;
  app.counterSamples = [];
  app.canRate = null;
  app.lastCounter = null;
}

function connect() {
  disconnect(true);
  if (settings.source === 'demo') {
    app.transport = new DemoTransport(onMessage, onLink);
  } else if (settings.source === 'bridge') {
    app.transport = new BridgeTransport(settings.wsUrl, onMessage, onLink);
    log(`Conectando a ${settings.wsUrl}…`);
  } else {
    return;
  }
  app.transport.connect();
}

function disconnect(silent = false) {
  if (app.transport) {
    const t = app.transport;
    app.transport = null;
    t.close();
    if (!silent) log('Desconectado.');
  }
  onLink('disconnected');
}

/* ------------------------------------------------------------------------ */
/* Renderizado                                                               */
/* ------------------------------------------------------------------------ */
function scheduleRender() {
  if (app.renderQueued) return;
  app.renderQueued = true;
  requestAnimationFrame(() => { app.renderQueued = false; render(); });
}

function setPill(id, cls, text) {
  const el = $(id);
  el.className = 'pill' + (cls ? ' ' + cls : '');
  el.querySelector('.pill-value').textContent = text;
}

function estimateVelocity() {
  const h = app.history;
  if (h.length < 2 || app.target === 'halted') return null;
  const last = h[h.length - 1];
  let i = h.length - 2;
  while (i > 0 && last.t - h[i].t < 300) i--;
  const dt = (last.t - h[i].t) / 1000;
  if (dt <= 0.05) return null;
  return (last.pos - h[i].pos) / dt;
}

function render() {
  const now = performance.now();
  const v = app.vars;
  const connected = app.link === 'connected';
  const halted = app.target === 'halted';
  const stale = connected && now - app.lastMsgT > STALE_MS;
  const hasData = connected && v.live_position !== undefined;

  /* --- Pills --- */
  setPill('link-pill',
    connected ? (stale ? 'warn' : 'ok') : app.link === 'connecting' ? 'busy' : '',
    connected ? (settings.source === 'demo' ? 'Demo' : 'OK') : app.link === 'connecting' ? 'Conectando…' : 'Desconectado');

  const tgtMap = { running: ['ok', 'RUNNING'], halted: ['halt', 'HALTED'], reset: ['warn', 'RESET'], unknown: ['', '—'] };
  const [tc, tt] = connected ? (tgtMap[app.target] || ['warn', app.target]) : ['', '—'];
  setPill('target-pill', tc, tt);

  const canDead = connected && app.target === 'running' && !stale && now - app.lastCounterChangeT > settings.canTimeoutMs;
  if (!connected || halted || app.canRate === null) setPill('can-pill', halted ? 'halt' : '', halted ? 'pausa' : '—');
  else setPill('can-pill', canDead ? 'err' : 'ok', canDead ? 'SIN DATOS' : `${app.canRate.toFixed(1)} Hz`);

  const isStudy = settings.source === 'fsm' || settings.source === 'matrix';

  /* --- Banners --- */
  $('halt-banner').classList.toggle('hidden', isStudy || !(connected && halted));
  $('can-banner').classList.toggle('hidden', isStudy || !canDead);
  $('stale-banner').classList.toggle('hidden', isStudy || !stale);

  /* --- KPI: posición --- */
  if (hasData) {
    $('kpi-pos').textContent = `${fmtSigned(v.live_position)}°`;
    const ticksVal = v.live_position_ticks !== undefined
      ? v.live_position_ticks
      : Math.round((v.live_position * TICKS_PER_GEAR_REV) / 360);
    if ($('kpi-ticks')) $('kpi-ticks').textContent = `Ticks: ${fmtSigned(ticksVal)} pulsos`;
    const turns = v.live_position / 360;
    $('kpi-pos-sub').textContent = `grados · ${turns.toFixed(2)} vueltas (2520 ticks = 360°)`;
    const vel = estimateVelocity();
    if (vel === null) $('kpi-vel').textContent = halted ? 'Vel: — (pausa)' : 'Vel: —';
    else {
      const s = `Vel: ${fmtSigned(Math.round(vel))} °/s · ${(vel / 360 * 60).toFixed(1)} RPM`;
      $('kpi-vel').textContent = s;
    }
  } else {
    $('kpi-pos').textContent = '—';
    if ($('kpi-ticks')) $('kpi-ticks').textContent = 'Ticks: —';
    $('kpi-vel').textContent = 'Vel: —';
  }

  /* --- KPI: estado motor --- */
  const badge = $('kpi-state');
  if (hasData) {
    const effSt = effectiveMotorState(v.live_motor_state, v.live_position);
    const si = stateInfo(effSt);
    badge.textContent = si.label;
    badge.className = 'state-badge ' + si.cls;
    $('kpi-state-desc').textContent = `${si.desc} (${effSt})`;
  } else {
    badge.textContent = '—';
    badge.className = 'state-badge';
    $('kpi-state-desc').innerHTML = '&nbsp;';
  }

  /* --- KPI: modo y FSM Master --- */
  const modeEl = $('kpi-mode');
  const mstBadge = $('kpi-master-fsm');
  if (hasData) {
    modeEl.textContent = modeName(v.system_mode);
    modeEl.className = 'kpi-value ' + (v.system_mode === MODE.REMOTE ? 'mode-remote' : 'mode-local');
    if (mstBadge) {
      const mVal = v.master_state !== undefined
        ? v.master_state
        : (canDead ? 3 : v.live_motor_state === 4 ? 4 : v.system_mode === MODE.REMOTE ? 2 : 1);
      const mi = masterStateInfo(mVal);
      mstBadge.textContent = `FSM: ${mi.label} (${mVal})`;
      mstBadge.className = 'mst-badge mono' + (mi.cls ? ' ' + mi.cls : '');
    }
    $('kpi-mode-sub').textContent = v.system_mode === MODE.REMOTE
      ? 'Control desde PC · pulsadores bloqueados (excepto BTN4)'
      : 'Pulsadores físicos BTN1..4 activos';
  } else {
    modeEl.textContent = '—';
    modeEl.className = 'kpi-value';
    if (mstBadge) {
      mstBadge.textContent = 'FSM: —';
      mstBadge.className = 'mst-badge mono';
    }
    $('kpi-mode-sub').innerHTML = '&nbsp;';
  }

  /* --- KPI: contador --- */
  if (hasData) {
    $('kpi-counter').textContent = fmtInt(v.telemetry_counter);
    $('kpi-counter-sub').textContent = halted ? 'congelado (pausa)'
      : app.canRate !== null ? `${app.canRate.toFixed(1)} tramas/s` : 'midiendo…';
  } else {
    $('kpi-counter').textContent = '—';
    $('kpi-counter-sub').innerHTML = '&nbsp;';
  }

  /* --- Controles --- */
  const isRemote = v.system_mode === MODE.REMOTE;
  $('mode-local').disabled = !connected;
  $('mode-remote').disabled = !connected;
  $('mode-local').className = 'seg' + (hasData && !isRemote ? ' on-local' : '');
  $('mode-remote').className = 'seg' + (hasData && isRemote ? ' on-remote' : '');

  document.querySelectorAll('.cmd').forEach((b) => {
    const n = Number(b.dataset.cmd);
    b.disabled = !connected || !isRemote || (app.pendingCmd && n !== 4);
    b.classList.toggle('sent', !!app.pendingCmd && app.pendingCmd.n === n);
  });

  if ($('dir-pos') && $('dir-neg')) {
    $('dir-pos').className = 'seg' + (app.stepDir === 1 ? ' on-dir-pos' : '');
    $('dir-neg').className = 'seg' + (app.stepDir === -1 ? ' on-dir-neg' : '');
  }

  const signPrefix = app.stepDir === 1 ? '+' : '−';
  document.querySelectorAll('.step-btn').forEach((b) => {
    const deg = Number(b.dataset.deg);
    const cmdNum = STEP_CMD_MAP[app.stepDir]?.[deg];
    const degLabel = b.querySelector('.step-deg');
    if (degLabel) degLabel.textContent = `${signPrefix}${deg}°`;
    b.disabled = !connected || !isRemote || !!app.pendingCmd;
    b.classList.toggle('sent', !!app.pendingCmd && app.pendingCmd.n === cmdNum);
  });

  const cs = $('cmd-status');
  if (app.pendingCmd) {
    const name = COMMANDS[app.pendingCmd.n];
    if (halted) { cs.textContent = `⏸ "${name}" escrito en RAM, pendiente: el micro está en pausa`; cs.className = 'cmd-status halted'; }
    else { cs.textContent = `⋯ "${name}" enviado (remote_cmd=${app.pendingCmd.n}), esperando al firmware`; cs.className = 'cmd-status pending'; }
  } else if (app.lastCmdResult && now - app.lastCmdResult.t < 4000) {
    cs.textContent = app.lastCmdResult.text;
    cs.className = 'cmd-status ' + app.lastCmdResult.cls;
  } else if (connected && hasData && !isRemote) {
    cs.textContent = 'Modo LOCAL: los comandos remotos están deshabilitados';
    cs.className = 'cmd-status';
  } else {
    cs.textContent = 'Sin comandos pendientes';
    cs.className = 'cmd-status';
  }

  /* --- Tabla de variables --- */
  for (const name of Object.keys(app.meta)) {
    const row = document.querySelector(`tr[data-var="${name}"]`);
    if (!row) continue;
    const val = v[name];
    const cell = row.querySelector('.v-val');
    let text = val === undefined || !connected ? '—' : String(val);
    if (name === 'live_motor_state' && val !== undefined && connected) {
      const effSt = effectiveMotorState(val, v.live_position);
      text = `${effSt} · ${stateInfo(effSt).label}`;
    }
    if (name === 'master_state' && val !== undefined && connected) text = `${val} · ${masterStateInfo(val).label}`;
    if (name === 'system_mode' && val !== undefined && connected) text = `${val} · ${modeName(val)}`;
    if (cell.textContent !== text) cell.textContent = text;
    cell.classList.toggle('flash', now - (app.varFlashT[name] || 0) < 350);
    const addr = app.meta[name].addr;
    row.querySelector('.v-addr').textContent = addr !== undefined ? fmtHex(addr) : '—';
  }

  $('btn-write').disabled = !connected;
  $('btn-connect').textContent = app.link === 'disconnected' ? 'Conectar' : 'Desconectar';
  $('rec-info').textContent = app.recording || app.csvRows.length ? `${fmtInt(app.csvRows.length)} filas` : '';
  $('btn-csv').disabled = app.csvRows.length === 0;

  drawChart(now);
}

/* ------------------------------------------------------------------------ */
/* Tabla de variables                                                        */
/* ------------------------------------------------------------------------ */
function buildVarsTable() {
  const tbody = $('vars-tbody');
  tbody.innerHTML = '';
  const select = $('write-var');
  const prevSel = select.value;
  select.innerHTML = '';
  for (const [name, m] of Object.entries(app.meta)) {
    const tr = document.createElement('tr');
    tr.dataset.var = name;
    if (m.writable) tr.classList.add('writable');
    tr.innerHTML = `<td class="v-name"></td><td class="v-type"></td><td class="v-val">—</td><td class="v-addr">—</td><td class="v-desc"></td>`;
    tr.querySelector('.v-name').textContent = name;
    tr.querySelector('.v-type').textContent = m.type || '?';
    tr.querySelector('.v-desc').textContent = m.desc || '';
    tr.addEventListener('click', () => {
      if (!m.writable) return;
      select.value = name;
      $('write-value').value = app.vars[name] ?? '';
      $('write-value').focus();
    });
    tbody.appendChild(tr);
    if (m.writable) {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      select.appendChild(opt);
    }
  }
  if (prevSel) select.value = prevSel;
}

/* ------------------------------------------------------------------------ */
/* Gráfico (canvas, sin dependencias)                                        */
/* ------------------------------------------------------------------------ */
function buildLegend() {
  $('chart-legend').innerHTML = Object.values(MOTOR_STATES)
    .map((s) => `<span><i style="background:${s.color}"></i>${s.label}</span>`).join('');
}

function niceStep(range, target) {
  const raw = range / target;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  const step = norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10;
  return step * mag;
}

function drawChart(now) {
  const canvas = $('chart');
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (w === 0 || h === 0) return;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const padL = 70, padR = 14, padT = 10, padB = 26;
  const pw = w - padL - padR, ph = h - padT - padB;
  const win = settings.chartWindow * 1000;
  const tEnd = app.chartPaused ? app.chartFreezeT : app.target === 'halted' ? app.haltStartT : now;
  const tStart = tEnd - win;

  const pts = app.history.filter((p) => p.t >= tStart - 200 && p.t <= tEnd);

  ctx.font = '11px Consolas, monospace';
  ctx.textBaseline = 'middle';

  if (pts.length < 2) {
    ctx.fillStyle = '#5d6b7d';
    ctx.textAlign = 'center';
    ctx.fillText(app.link === 'connected' ? 'Esperando datos…' : 'Sin conexión', padL + pw / 2, padT + ph / 2);
    return;
  }

  let min = Infinity, max = -Infinity;
  for (const p of pts) { if (p.pos < min) min = p.pos; if (p.pos > max) max = p.pos; }
  if (max - min < 20) { const c = (max + min) / 2; min = c - 10; max = c + 10; }
  const pad = (max - min) * 0.1;
  min -= pad; max += pad;

  const xOf = (t) => padL + ((t - tStart) / win) * pw;
  const yOf = (p) => padT + (1 - (p - min) / (max - min)) * ph;

  /* Grilla horizontal */
  const yStep = niceStep(max - min, 5);
  ctx.textAlign = 'right';
  for (let y = Math.ceil(min / yStep) * yStep; y <= max; y += yStep) {
    const py = yOf(y);
    ctx.strokeStyle = Math.abs(y) < yStep / 2 ? '#3a4a5e' : '#1f2935';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(padL, py); ctx.lineTo(padL + pw, py); ctx.stroke();
    ctx.fillStyle = '#6c7a8d';
    ctx.fillText(fmtInt(Math.round(y)), padL - 8, py);
  }

  /* Grilla vertical (segundos relativos) */
  const xStep = niceStep(settings.chartWindow, 6);
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (let s = 0; s <= settings.chartWindow + 1e-6; s += xStep) {
    const px = padL + pw - (s / settings.chartWindow) * pw;
    ctx.strokeStyle = '#1a232e';
    ctx.beginPath(); ctx.moveTo(px, padT); ctx.lineTo(px, padT + ph); ctx.stroke();
    ctx.fillStyle = '#6c7a8d';
    ctx.fillText(s === 0 ? 'ahora' : `-${+s.toFixed(1)} s`, px, padT + ph + 7);
  }

  /* Línea coloreada por estado del motor (con franja de tolerancia Home ±5°) */
  ctx.save();
  ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
  if (max >= -HOMED_TOLERANCE_DEG && min <= HOMED_TOLERANCE_DEG) {
    const yTop = yOf(HOMED_TOLERANCE_DEG);
    const yBot = yOf(-HOMED_TOLERANCE_DEG);
    ctx.fillStyle = 'rgba(46, 204, 113, 0.08)';
    ctx.fillRect(padL, yTop, pw, yBot - yTop);
  }
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  let i = 0;
  while (i < pts.length - 1) {
    const st = pts[i].st;
    ctx.strokeStyle = stateInfo(st).color;
    ctx.beginPath();
    ctx.moveTo(xOf(pts[i].t), yOf(pts[i].pos));
    let j = i + 1;
    for (; j < pts.length; j++) {
      ctx.lineTo(xOf(pts[j].t), yOf(pts[j].pos));
      if (pts[j].st !== st) break;
    }
    ctx.stroke();
    i = j;
  }
  ctx.restore();

  /* Marco */
  ctx.strokeStyle = '#263241';
  ctx.strokeRect(padL + 0.5, padT + 0.5, pw, ph);

  if (app.chartPaused || app.target === 'halted') {
    ctx.fillStyle = app.target === 'halted' ? '#b38cff' : '#f5b942';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillText(app.target === 'halted' ? '⏸ MICRO EN PAUSA' : '⏸ GRÁFICO CONGELADO', padL + pw - 8, padT + 8);
  }
}

/* ------------------------------------------------------------------------ */
/* CSV                                                                       */
/* ------------------------------------------------------------------------ */
function toggleRecording() {
  app.recording = !app.recording;
  const b = $('btn-rec');
  if (app.recording) {
    app.csvRows = [];
    b.textContent = '■ Detener';
    b.classList.add('rec-on');
    log('Grabación CSV iniciada.', 'info');
  } else {
    b.textContent = '● Grabar CSV';
    b.classList.remove('rec-on');
    log(`Grabación CSV detenida (${app.csvRows.length} filas).`, 'info');
  }
  scheduleRender();
}

function downloadCSV() {
  const header = ['timestamp_iso', 't_ms_puente', 'target', 'live_position_deg', 'live_position_ticks', 'live_motor_state',
    'master_state', 'telemetry_counter', 'system_mode', 'remote_cmd'];
  const lines = [header.join(';'), ...app.csvRows.map((r) => r.map((x) => x ?? '').join(';'))];
  const blob = new Blob([lines.join('\r\n')], { type: 'text/csv;charset=utf-8' });
  const a = document.createElement('a');
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  a.href = URL.createObjectURL(blob);
  a.download = `telemetria_${stamp}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

/* ------------------------------------------------------------------------ */
/* Vistas de estudio: 2. Máquinas de Estados (FSM) y 3. Matriz de Errores    */
/* ------------------------------------------------------------------------ */
const FSM_SLAVE_DETAILS = [
  {
    id: 0, code: 'STATE_IDLE', cls: 'is-muted', badge: 'b-muted',
    short: 'Detenido fuera de Home (> ±8°)',
    entry: ['Motor_SetSpeed(0) (freno activo)', 'LED PC13 apagado (GPIO_PIN_SET)', 'Ocurre tras STOP o giro relativo fuera de ±56 ticks de Home'],
    accepts: ['BTN1 (0x01) → STATE_SCANNING', 'BTN2 (0x02) → Calibrar Home (STATE_HOMED)', 'BTN3 (0x03) → Ir a Home (STATE_HOMING)', 'Giros 0x05..0x0C → STATE_POSITIONING'],
    faults: ['Sin movimiento: no dispara límites ni timeouts'],
    exit: ['Cualquier comando de movimiento o calibración'],
  },
  {
    id: 1, code: 'STATE_SCANNING', cls: 'is-info', badge: 'b-info',
    short: 'Barrido oscilante ±2 vueltas (±5040 ticks)',
    entry: ['Guarda leg_start_ticks = total_encoder_ticks', 'Inicia fase SCAN_PHASE_FORWARD con PWM = 700', 'LED PC13 encendido fijo'],
    accepts: ['BTN2 → Calibrar Home aquí', 'BTN3 → Ir a Home', 'BTN4 → STOP inmediato', 'Giros 0x05..0x0C → STATE_POSITIONING'],
    faults: ['Límite mecánico ≥ 25000 ticks → STATE_ERROR_LIMIT (4)', '⚠ NO tiene timeout: si se desconecta Enc A o falta Vmotor, queda esperando sin error'],
    exit: ['BTN4 (STOP), BTN2, BTN3, paso angular o falla de límite'],
  },
  {
    id: 2, code: 'STATE_HOMED', cls: 'is-ok', badge: 'b-ok',
    short: 'Detenido dentro de zona Home (±8° / ±56 ticks)',
    entry: ['Motor_SetSpeed(0) (freno activo)', 'LED PC13 apagado', 'Estado inicial al encender o tras Slave_CalibrateHome()'],
    accepts: ['BTN1 → Barrido', 'BTN2 → Recalibrar cero aquí', 'BTN3 → Reafirmar Home', 'Giros 0x05..0x0C → STATE_POSITIONING'],
    faults: ['En reposo no hay fallas activas'],
    exit: ['BTN1 o cualquier comando de giro relativo'],
  },
  {
    id: 3, code: 'STATE_HOMING', cls: 'is-warn', badge: 'b-warn',
    short: 'Retornando activamente a 0° (home_position)',
    entry: ['target_position = home_position', 'Inicia go_home_start_time = HAL_GetTick()', 'LED PC13 encendido fijo'],
    accepts: ['BTN2 → Calibrar cero en el acto', 'BTN4 → STOP inmediato', 'Giros 0x05..0x0C → cambia a STATE_POSITIONING'],
    faults: ['Timeout > 30000 ms sin llegar a ±5 ticks → STATE_ERROR_LIMIT (4)', 'Límite ≥ 25000 ticks → STATE_ERROR_LIMIT (4)', 'Error crece > +30 ticks → auto-invierte motor_dir_sign'],
    exit: ['Llegada a |error| ≤ 5 ticks → STATE_HOMED'],
  },
  {
    id: 4, code: 'STATE_ERROR_LIMIT', cls: 'is-err', badge: 'b-err',
    short: 'Falla crítica: límite ≥ 25000 ticks o timeout > 30 s',
    entry: ['Motor_SetSpeed(0) (freno inmediato)', 'LED PC13 parpadea rápido cada 100 ms', 'Reporta estado = 4 en CAN 0x030 (Maestro pasa a MASTER_SLAVE_ERROR)'],
    accepts: ['BTN2 (0x02) → Recalibrar Home y salir a STATE_HOMED', 'BTN1 (0x01) → Limpiar falla y pasar a reposo', 'BTN4 (0x04) → Freno y pasar a reposo'],
    faults: ['Bloquea BTN3 y todos los giros relativos hasta limpiar la falla'],
    exit: ['Solo mediante BTN1, BTN2 o BTN4'],
  },
  {
    id: 5, code: 'STATE_POSITIONING', cls: 'is-cyan', badge: 'b-cyan',
    short: 'Posicionamiento relativo (±30°/90°/180°/360°)',
    entry: ['target_position = total_encoder_ticks + step_req', 'Comparte Slave_RunPositionControl() con STATE_HOMING', 'LED PC13 encendido fijo'],
    accepts: ['Nuevos giros 0x05..0x0C acumulan sobre target_position', 'BTN2 → Fija cero aquí', 'BTN3 → Redirige a STATE_HOMING', 'BTN4 → STOP'],
    faults: ['Timeout > 30000 ms → STATE_ERROR_LIMIT (4)', 'Límite ≥ 25000 ticks → STATE_ERROR_LIMIT (4)', 'Error crece > +30 ticks → auto-invierte motor_dir_sign'],
    exit: ['Llegada a |error| ≤ 5 ticks → STATE_HOMED o STATE_IDLE'],
  },
];

const FSM_MASTER_DETAILS = [
  {
    id: 0, code: 'MASTER_INIT', cls: 'is-muted', badge: 'b-muted',
    short: 'Inicialización de GPIO, CAN y espera de enlace',
    entry: ['3 parpadeos iniciales de 100 ms en PC13', 'Inicia CAN1 + filtro + interrupción FIFO0'],
    accepts: ['Transiciona en el primer ciclo del while(1)'],
    faults: ['Si falla HAL_CAN_Start o filtro → Error_Handler() (bloqueo total)'],
    exit: ['Pasa automáticamente a MASTER_LOCAL o MASTER_REMOTE según system_mode'],
  },
  {
    id: 1, code: 'MASTER_LOCAL', cls: 'is-ok', badge: 'b-ok',
    short: 'Operación normal con pulsadores físicos BTN1..BTN4',
    entry: ['current_led_state = 0', 'PC13 apagado (refleja PB0=0 en Placa B)'],
    accepts: ['BTN1..BTN4 físicos envían CAN 0x010', 'BTN6 / system_mode=1 cambia a MASTER_REMOTE', 'Descarta remote_cmd desde PC/HMI'],
    faults: ['Sin trama 0x030 > 500 ms → MASTER_CAN_FAULT (3)', 'live_motor_state == 4 → MASTER_SLAVE_ERROR (4)'],
    exit: ['Cambio a Remoto o disparo de falla CAN / Esclavo'],
  },
  {
    id: 2, code: 'MASTER_REMOTE', cls: 'is-info', badge: 'b-info',
    short: 'Control desde PC / HMI (remote_cmd 1..12) + BTN4 físico',
    entry: ['current_led_state = 1', 'PC13 encendido fijo (refleja PB0=1 en Placa B)'],
    accepts: ['remote_cmd (1..12) enviado por ST-Link/HMI', 'BTN4 físico siempre activo por seguridad de banco', 'BTN5 / system_mode=0 vuelve a MASTER_LOCAL'],
    faults: ['Sin trama 0x030 > 500 ms → MASTER_CAN_FAULT (3)', 'live_motor_state == 4 → MASTER_SLAVE_ERROR (4)'],
    exit: ['Cambio a Local o disparo de falla CAN / Esclavo'],
  },
  {
    id: 3, code: 'MASTER_CAN_FAULT', cls: 'is-err', badge: 'b-err',
    short: 'Pérdida de enlace CAN (> 500 ms sin telemetría 0x030)',
    entry: ['Descarta cualquier remote_cmd pendiente', 'Inicia parpadeo rápido de PC13 cada 120 ms'],
    accepts: ['Solo supervisa llegada de nuevas tramas 0x030', 'Tiene mayor prioridad que MASTER_SLAVE_ERROR'],
    faults: ['Bloquea operación hasta recuperar la comunicación CAN'],
    exit: ['Recuperación automática apenas vuelve 0x030 (≤ 500 ms); reenvía 0x020'],
  },
  {
    id: 4, code: 'MASTER_SLAVE_ERROR', cls: 'is-warn', badge: 'b-warn',
    short: 'Placa B reportó STATE_ERROR_LIMIT (live_motor_state == 4)',
    entry: ['Inicia parpadeo de advertencia en PC13 cada 250 ms'],
    accepts: ['Solo deja pasar comandos de desbloqueo: BTN1 (1), BTN2 (2) y BTN4 (4)', 'Descarta BTN3 (3) y giros (5..12)'],
    faults: ['Si además cae el bus CAN > 500 ms → escala a MASTER_CAN_FAULT (3)'],
    exit: ['Automático cuando Placa B vuelve a reportar live_motor_state != 4'],
  },
];

const ERROR_MATRIX_ROWS = [
  { sub: 'slave', name: 'STATE_ERROR_LIMIT (4) por Límite Mecánico', cause: 'abs(total_encoder_ticks - home_position) >= 25000 ticks durante movimiento.', effect: 'Motor_SetSpeed(0) inmediato; bloquea nuevos giros hasta limpiar falla.', led: 'B: PC13 parpadea cada 100 ms · HMI: ERROR (4)', rec: 'Enviar BTN2 (Set Home) o BTN1/BTN4.' },
  { sub: 'slave', name: 'STATE_ERROR_LIMIT (4) por Timeout de Posición', cause: '(HAL_GetTick() - go_home_start_time) > 30000 ms en HOMING o POSITIONING sin entrar en ±5 ticks.', effect: 'Frena el motor y corta el intento de posicionamiento.', led: 'B: PC13 parpadea cada 100 ms · HMI: ERROR (4)', rec: 'Revisar traba/energía/encoder y pulsar BTN2, BTN1 o BTN4.' },
  { sub: 'slave', name: 'Auto-corrección Anti-embalamiento (Anomalía)', cause: 'abs_err > go_home_initial_error + 30 ticks en Slave_RunPositionControl().', effect: 'Invierte motor_dir_sign (+1 ↔ -1) automáticamente para evitar que el error siga creciendo.', led: 'Sin alerta visual (se corrige en vuelo).', rec: 'Automática en el ciclo siguiente.' },
  { sub: 'slave', name: 'Error_Handler() en Placa B', cause: 'Falla de inicialización HAL (SystemClock, TIM2, TIM3, CAN_Start, CAN_Filter).', effect: '__disable_irq() y bucle infinito while(1); placa congelada.', led: 'Sin parpadeo; deja de emitir CAN 0x030 (A ve CAN_FAULT).', rec: 'Reinicio físico (Reset / ciclo de alimentación).' },
  { sub: 'master', name: 'MASTER_CAN_FAULT (3)', cause: '> 500 ms sin recibir tramas CAN 0x030 desde Placa B (CAN_TELEMETRY_TIMEOUT_MS).', effect: 'Descarta comandos remotos; prioridad superior a MASTER_SLAVE_ERROR.', led: 'A: PC13 parpadea rápido cada 120 ms · HMI: CAN_FAULT (3)', rec: 'Automática al reaparecer 0x030; reenvía 0x020.' },
  { sub: 'master', name: 'MASTER_SLAVE_ERROR (4)', cause: 'Trama CAN 0x030 trae live_motor_state == 4.', effect: 'Solo permite comandos 1 (BTN1), 2 (BTN2) y 4 (STOP); descarta el resto.', led: 'A: PC13 parpadea cada 250 ms · HMI: SLAVE_ERROR (4)', rec: 'Automática cuando Placa B sale de estado 4.' },
  { sub: 'master', name: 'Fallo de ACK en Transmisión CAN', cause: 'CAN_SendMessage() falla al encolar o no recibe ACK en 25 ms.', effect: 'Aborta el buzón TX pendiente y devuelve 0.', led: 'A: Indicate_CAN_Result(0) parpadea PC13 4 veces cada 40 ms.', rec: 'Se restablece tras la ráfaga; reintentar comando.' },
  { sub: 'master', name: 'Error_Handler() en Placa A', cause: 'Falla de inicialización HAL (CAN_Filter, HAL_CAN_Start, Notification, Clock).', effect: '__disable_irq() y bucle infinito while(1).', led: 'Placa congelada; puente Python deja de ver cambios.', rec: 'Reinicio físico (Reset).' },
  { sub: 'hmi', name: 'Banner CAN SIN DATOS (canDead)', cause: 'CPU corriendo, WS fresco, pero telemetry_counter sin cambios > canTimeoutMs (1000 ms por defecto).', effect: 'Pone pill CAN en rojo (SIN DATOS) y muestra banner de alerta.', led: 'HMI: Banner rojo de pérdida de telemetría CAN.', rec: 'Desaparece apenas vuelve a incrementar telemetry_counter.' },
  { sub: 'hmi', name: 'CPU en Pausa (HALTED)', cause: 'El depurador ST-Link / OpenOCD pausó el Cortex-M3 (target == halted).', effect: 'Variables congeladas; comandos escritos en RAM quedan pendientes sin ejecutarse.', led: 'HMI: Pill violeta HALTED y banner de CPU en pausa.', rec: 'Reanudar ejecución (Resume / F8) desde CubeIDE o puente.' },
  { sub: 'hmi', name: 'Datos Viejos / Enlace Caído (STALE)', cause: '> 2000 ms (STALE_MS) sin mensajes desde bridge.py por WebSocket.', effect: 'Pill de puente pasa a amarillo y alerta desconexión.', led: 'HMI: Banner de enlace estancado.', rec: 'Reconecta automáticamente cada 2 s o reiniciar bridge.py.' },
  { sub: 'hmi', name: 'CMD_TIMEOUT / WRITE_TIMEOUT', cause: 'remote_cmd != 0 tras 3000 ms o falta ACK del puente en 2000 ms.', effect: 'Libera el botón pendiente y registra error rojo en la consola HMI.', led: 'HMI: Log rojo "no fue procesado por el firmware (timeout)".', rec: 'Verificar modo REMOTE y estado RUNNING del micro.' },
  { sub: 'hw', name: 'Canal A (PA0) desconectado / Sin Vmotor', cause: 'No llegan flancos a TIM2_CH1 (modo TI1): delta = 0 aunque haya PWM.', effect: 'En HOMING/POSITIONING dispara Error 4 a los 30 s. En SCANNING NO dispara falla (queda esperando 5040 ticks).', led: 'En SCANNING no da alerta. En POS/HOMING da Error 4 tras 30 s.', rec: 'Reconectar señal/energía; usar BTN4 (STOP) si quedó en barrido.' },
  { sub: 'hw', name: 'Canal B (PA1) desconectado', cause: 'TIM2_CH2 queda flotante (GPIO_NOPULL); cuenta flancos de A sin referencia de fase B.', effect: 'Conteo con sentido errático o fijo; puede disparar anti-inversión, freno falso, Error 4 por límite/timeout o seguir sin alerta en barrido.', led: 'Variable según comportamiento del pin flotante.', rec: 'Reconectar Canal B en PA1 y recalibrar Home (BTN2).' },
];

function showFSMDetail(st, prefix) {
  $('fsm-detail-title').textContent = `${prefix} · ${st.code} (Valor = ${st.id})`;
  const b = $('fsm-detail-badge');
  b.textContent = st.short;
  b.className = `m-badge ${st.badge}`;

  const mkList = (arr) => `<ul>${arr.map((x) => `<li>${x}</li>`).join('')}</ul>`;
  $('fsm-detail-body').innerHTML = `
    <div class="fsm-detail-cell"><h4>Entrada y Salidas / LEDs</h4>${mkList(st.entry)}</div>
    <div class="fsm-detail-cell"><h4>Comandos Aceptados</h4>${mkList(st.accepts)}</div>
    <div class="fsm-detail-cell"><h4>Fallas o Alertas Posibles</h4>${mkList(st.faults)}</div>
    <div class="fsm-detail-cell"><h4>Condiciones de Salida</h4>${mkList(st.exit)}</div>`;
}

function renderErrorMatrix() {
  const activeBtn = document.querySelector('#view-matrix [data-filter].active');
  const filter = activeBtn ? activeBtn.dataset.filter : 'all';
  const q = $('matrix-search').value.trim().toLowerCase();

  const subNames = {
    slave: '<span class="m-badge b-info">Placa B</span>',
    master: '<span class="m-badge b-warn">Placa A</span>',
    hmi: '<span class="m-badge b-halt">HMI / SWD</span>',
    hw: '<span class="m-badge b-muted">Hardware</span>',
  };

  const rows = ERROR_MATRIX_ROWS.filter((r) => {
    if (filter !== 'all' && r.sub !== filter) return false;
    if (!q) return true;
    return `${r.name} ${r.cause} ${r.effect} ${r.led} ${r.rec}`.toLowerCase().includes(q);
  });

  $('matrix-count').textContent = `${rows.length} de ${ERROR_MATRIX_ROWS.length} casos`;
  $('matrix-tbody').innerHTML = rows.map((r) => `
    <tr>
      <td>${subNames[r.sub]}</td>
      <td><strong>${r.name}</strong></td>
      <td>${r.cause}</td>
      <td>${r.effect}</td>
      <td>${r.led}</td>
      <td>${r.rec}</td>
    </tr>`).join('');
}

function initStudyViews() {
  const renderList = (arr, targetId, prefix) => {
    const cont = $(targetId);
    cont.innerHTML = '';
    arr.forEach((st) => {
      const d = document.createElement('div');
      d.className = `fsm-node ${st.cls}` + (prefix.includes('Placa B') && st.id === 4 ? ' selected' : '');
      d.innerHTML = `<div class="fsm-node-head">
        <span class="fsm-node-code mono">${st.code}</span>
        <span class="tag mono">${st.id}</span></div>
        <div class="fsm-node-short">${st.short}</div>`;
      d.addEventListener('click', () => {
        document.querySelectorAll('.fsm-node').forEach((n) => n.classList.remove('selected'));
        d.classList.add('selected');
        showFSMDetail(st, prefix);
      });
      cont.appendChild(d);
    });
  };

  renderList(FSM_SLAVE_DETAILS, 'fsm-b-nodes', 'Placa B (Esclavo)');
  renderList(FSM_MASTER_DETAILS, 'fsm-a-nodes', 'Placa A (Maestro)');
  showFSMDetail(FSM_SLAVE_DETAILS[4], 'Placa B (Esclavo)');

  document.querySelectorAll('#view-matrix [data-filter]').forEach((b) => {
    b.addEventListener('click', () => {
      document.querySelectorAll('#view-matrix [data-filter]').forEach((x) => x.classList.remove('active'));
      b.classList.add('active');
      renderErrorMatrix();
    });
  });
  $('matrix-search').addEventListener('input', renderErrorMatrix);
  renderErrorMatrix();
}

/* ------------------------------------------------------------------------ */
/* Inicialización de la UI                                                   */
/* ------------------------------------------------------------------------ */
function applySourceUI() {
  const src = settings.source;
  const isLive = src === 'bridge' || src === 'demo';
  $('source-select').value = src;
  $('ws-url').classList.toggle('hidden', src !== 'bridge');
  $('btn-connect').classList.toggle('hidden', !isLive);
  $('demo-card').classList.toggle('hidden', src !== 'demo');
  $('hmi-main').classList.toggle('hidden', !isLive);
  $('view-fsm').classList.toggle('hidden', src !== 'fsm');
  $('view-matrix').classList.toggle('hidden', src !== 'matrix');
}

function bindUI() {
  $('ws-url').value = settings.wsUrl;
  $('chart-window').value = String(settings.chartWindow);

  $('source-select').addEventListener('change', (e) => {
    settings.source = e.target.value;
    saveSettings();
    applySourceUI();
    if (settings.source === 'bridge' || settings.source === 'demo') {
      connect();
    } else {
      disconnect(true);
      scheduleRender();
    }
  });

  $('ws-url').addEventListener('change', (e) => {
    settings.wsUrl = e.target.value.trim();
    saveSettings();
  });

  $('btn-connect').addEventListener('click', () => {
    if (app.link === 'disconnected') {
      settings.wsUrl = $('ws-url').value.trim();
      saveSettings();
      connect();
    } else {
      disconnect();
    }
  });

  $('mode-local').addEventListener('click', () => setMode(MODE.LOCAL));
  $('mode-remote').addEventListener('click', () => setMode(MODE.REMOTE));
  document.querySelectorAll('.cmd').forEach((b) =>
    b.addEventListener('click', () => sendCommand(Number(b.dataset.cmd))));

  $('dir-pos')?.addEventListener('click', () => {
    app.stepDir = 1;
    scheduleRender();
  });
  $('dir-neg')?.addEventListener('click', () => {
    app.stepDir = -1;
    scheduleRender();
  });
  document.querySelectorAll('.step-btn').forEach((b) =>
    b.addEventListener('click', () => {
      const deg = Number(b.dataset.deg);
      const cmdNum = STEP_CMD_MAP[app.stepDir]?.[deg];
      if (cmdNum) sendCommand(cmdNum);
    }));

  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && !document.querySelector('dialog[open]') &&
        (settings.source === 'bridge' || settings.source === 'demo')) {
      e.preventDefault();
      sendCommand(4);
    }
  });

  $('btn-write').addEventListener('click', () => {
    const name = $('write-var').value;
    const raw = $('write-value').value;
    if (!name || raw === '' || !Number.isFinite(Number(raw))) { log('Valor inválido para escribir.', 'warn'); return; }
    writeVar(name, Number(raw));
  });
  $('write-value').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('btn-write').click(); });

  $('chart-window').addEventListener('change', (e) => {
    settings.chartWindow = Number(e.target.value);
    saveSettings();
    scheduleRender();
  });
  $('btn-chart-pause').addEventListener('click', () => {
    app.chartPaused = !app.chartPaused;
    app.chartFreezeT = performance.now();
    $('btn-chart-pause').textContent = app.chartPaused ? '▶ Reanudar' : '⏸ Congelar';
    $('btn-chart-pause').classList.toggle('active', app.chartPaused);
    scheduleRender();
  });

  $('btn-rec').addEventListener('click', toggleRecording);
  $('btn-csv').addEventListener('click', downloadCSV);
  $('btn-clear-log').addEventListener('click', () => { $('log-list').innerHTML = ''; });

  /* Ajustes */
  $('btn-settings').addEventListener('click', () => {
    $('set-ppr').value = settings.ppr;
    $('set-can-timeout').value = settings.canTimeoutMs;
    $('settings-dialog').showModal();
  });
  $('settings-dialog').addEventListener('close', () => {
    if ($('settings-dialog').returnValue !== 'save') return;
    settings.ppr = Math.max(0, Number($('set-ppr').value) || 0);
    settings.canTimeoutMs = Math.max(200, Number($('set-can-timeout').value) || DEFAULTS.canTimeoutMs);
    saveSettings();
    scheduleRender();
  });

  /* Simulador */
  const sim = () => (app.transport instanceof DemoTransport ? app.transport : null);
  $('btn-sim-halt').addEventListener('click', () => {
    const s = sim(); if (!s) return;
    s.setHalted(!s.halted);
    $('btn-sim-halt').textContent = s.halted ? '▶ Reanudar (resume)' : '⏸ Breakpoint (halt)';
    $('btn-sim-halt').classList.toggle('active', s.halted);
  });
  $('btn-sim-canfault').addEventListener('click', () => {
    const s = sim(); if (!s) return;
    s.setCanFault(!s.canFault);
    $('btn-sim-canfault').textContent = s.canFault ? '⚡ Restaurar bus CAN' : '⚡ Cortar bus CAN';
    $('btn-sim-canfault').classList.toggle('active', s.canFault);
    log(s.canFault ? '[sim] Bus CAN cortado.' : '[sim] Bus CAN restaurado.', 'warn');
  });
  $('btn-sim-error').addEventListener('click', () => { sim()?.injectError(); log('[sim] ERROR inyectado en placa B.', 'warn'); });
  $('btn-sim-btn5').addEventListener('click', () => { sim()?.physicalButton(5); log('[sim] BTN5 físico presionado.'); });
  $('btn-sim-btn6').addEventListener('click', () => { sim()?.physicalButton(6); log('[sim] BTN6 físico presionado.'); });
  $('btn-sim-btn4').addEventListener('click', () => { sim()?.physicalButton(4); log('[sim] BTN4 físico (Stop) presionado.'); });

  window.addEventListener('resize', scheduleRender);
}

/* Vigilancia periódica: timeouts de escritura, datos viejos, alarmas */
function housekeeping() {
  const now = performance.now();
  for (const [id, pw] of app.pendingWrites) {
    if (now - pw.t > WRITE_TIMEOUT_MS) {
      app.pendingWrites.delete(id);
      log(`✗ Sin confirmación del puente para ${pw.name} = ${pw.value}`, 'err');
      if (pw.name === 'remote_cmd') app.pendingCmd = null;
    }
  }
  checkPendingCmd(now);
  scheduleRender();
}

function init() {
  buildLegend();
  buildVarsTable();
  initStudyViews();
  bindUI();
  applySourceUI();
  log('HMI iniciada. Fuente: ' + (settings.source === 'demo' ? 'simulador' : `puente ${settings.wsUrl}`));
  if (settings.source === 'bridge' || settings.source === 'demo') {
    connect();
  } else {
    scheduleRender();
  }
  setInterval(housekeeping, 250);
}

init();
