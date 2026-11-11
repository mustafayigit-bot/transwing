/**
 * Estimated telemetry: turns the "truth" channels of a 6-DOF mission into the messages an
 * ArduPilot (Plane / QuadPlane) flight controller would log — with sensor noise, bias, drift and
 * rotor-induced vibration. Message names, fields and units follow the ArduPilot DataFlash log.
 *
 * Frames: IMU in body FRD (x forward, y right, z down); positions as lat / lon / altitude AMSL.
 * Everything here is deterministic for a given seed.
 */
import type { Product } from '../data/schema';
import { PHASES, type MissionTrace } from '../physics/fdm/mission';
import { gaussian } from '../physics/wind';

export interface SensorSpec {
  accelNoiseMs2: number;
  accelBiasMs2: number;
  gyroNoiseRadS: number;
  gyroBiasRadS: number;
  /** in-plane (1/rev imbalance) vibration amplitude per rotor at the reference rpm, m/s² */
  vib1PMs2: number;
  vibRefRpm: number;
  /** axial (blade-pass) vibration amplitude per rotor per 100 N of thrust, m/s² */
  vibAxialMs2Per100N: number;
  /** buffet when the wing is stalled / folding at speed, m/s² at q = 300 Pa */
  buffetMs2: number;
  gpsHorizSigmaM: number;
  gpsVertSigmaM: number;
  gpsVelSigmaMs: number;
  baroNoiseM: number;
  baroDriftM: number;
  airspeedNoiseMs: number;
}

/** ASSUMPTION: generic values for a Cube Orange class autopilot on a soft-mounted carrier board. */
export const DEFAULT_SENSORS: SensorSpec = {
  accelNoiseMs2: 0.08, accelBiasMs2: 0.05, gyroNoiseRadS: 0.003, gyroBiasRadS: 0.002,
  vib1PMs2: 5.5, vibRefRpm: 3500, vibAxialMs2Per100N: 3.5, buffetMs2: 6,
  gpsHorizSigmaM: 0.8, gpsVertSigmaM: 1.5, gpsVelSigmaMs: 0.08, baroNoiseM: 0.08, baroDriftM: 0.5, airspeedNoiseMs: 0.3,
};

export interface TelemetryOptions {
  seed: number;
  homeLatDeg: number;
  homeLonDeg: number;
  homeAltMslM: number;
  /** UTC start time of the flight (ms since epoch) */
  startEpochMs: number;
  imuRateHz: number;
  sensors: SensorSpec;
}

export const DEFAULT_TELEMETRY: Omit<TelemetryOptions, 'seed' | 'homeAltMslM'> = {
  homeLatDeg: 39.9208, homeLonDeg: 32.8541, startEpochMs: Date.UTC(2026, 10, 11, 9, 0, 0), imuRateHz: 50, sensors: DEFAULT_SENSORS,
};

/** One DataFlash message type: FMT definition + rows (first column is always TimeUS). */
export interface Stream {
  name: string;
  format: string;
  labels: string[];
  rows: (number | string)[][];
}

export interface Telemetry {
  productId: string;
  options: TelemetryOptions;
  streams: Stream[];
  /** sensor-derived channels sampled at the trace rate (for charts and statistics) */
  ch: Record<string, number[]>;
  assumptions: string[];
}

const DEG = Math.PI / 180;
const R_EARTH = 6378137;

/** ArduPilot Plane flight-mode numbers used in the log. */
export const AP_MODE = { AUTO: 10, QHOVER: 18, QLOITER: 19, QLAND: 20 };

export function ascii(s: string) {
  return s
    .replace(/[çÇ]/g, 'c').replace(/[ğĞ]/g, 'g').replace(/[ıİ]/g, 'i').replace(/[öÖ]/g, 'o').replace(/[şŞ]/g, 's').replace(/[üÜ]/g, 'u')
    .replace(/[^\x20-\x7e]/g, '?');
}

export function synthesizeTelemetry(trace: MissionTrace, product: Product, options: Partial<TelemetryOptions> = {}): Telemetry {
  const opt: TelemetryOptions = { ...DEFAULT_TELEMETRY, seed: trace.config.seed, homeAltMslM: trace.config.env.altitudeM, ...options };
  const sp = opt.sensors;
  const c = trace.ch;
  const n = c.t.length;
  const nR = trace.nRotors;
  const rnd = gaussian(opt.seed * 7919 + 17);
  const maxFold = product.geometry.hinge.maxFoldDeg;
  const bat = product.fdm?.battery;
  const us = (t: number) => Math.round(t * 1e6);
  const dtTrace = n > 1 ? c.t[1] - c.t[0] : 0.1;

  // constant biases for this flight
  const accBias = [0, 1, 2].map(() => sp.accelBiasMs2 * rnd());
  const gyrBias = [0, 1, 2].map(() => sp.gyroBiasRadS * rnd());
  const phase1 = Array.from({ length: nR }, () => rnd() * Math.PI);
  const phase2 = Array.from({ length: nR }, () => rnd() * Math.PI);

  /** vibration amplitudes per axis (FRD) from rotor state at trace sample i */
  const vibAmp = (i: number) => {
    const f = Math.min(1, Math.max(0, (c.fold[i] ?? 0) / maxFold)); // 1 = hover
    const inPlane: number[] = [], axial: number[] = [], freq: number[] = [];
    for (let k = 0; k < nR; k++) {
      const rpm = c[`rpm${k + 1}`][i] ?? 0;
      inPlane.push(sp.vib1PMs2 * (rpm / sp.vibRefRpm) ** 2);
      axial.push((sp.vibAxialMs2Per100N * Math.max(0, c[`thrust${k + 1}`][i] ?? 0)) / 100);
      freq.push(rpm / 60);
    }
    const q = 0.5 * (c.rho[i] ?? 1.225) * (c.ias[i] ?? 0) ** 2;
    const buffet = (sp.buffetMs2 * (c.stall[i] ?? 0) * Math.min(2, q / 300)) * ((c.ias[i] ?? 0) > 6 ? 1 : 0);
    return { f, inPlane, axial, freq, buffet };
  };
  /** RMS vibration per axis (what ArduPilot reports as VIBE) */
  const vibRms = (i: number) => {
    const v = vibAmp(i);
    let x = 0, y = 0, z = 0;
    for (let k = 0; k < nR; k++) {
      const ip2 = (v.inPlane[k] ** 2) / 2, ax2 = (v.axial[k] ** 2) / 2;
      // hover: rotor plane = body x–y, thrust axis = z; cruise: rotor plane = y–z, thrust axis = x
      x += v.f * ip2 / 2 + (1 - v.f) * ax2;
      y += ip2 / 2;
      z += v.f * ax2 + (1 - v.f) * ip2 / 2;
    }
    const b2 = v.buffet ** 2 / 3, n2 = sp.accelNoiseMs2 ** 2;
    return [Math.sqrt(x + b2 + n2), Math.sqrt(y + b2 + n2), Math.sqrt(z + b2 + n2)];
  };

  const ch: Record<string, number[]> = { vibeX: [], vibeY: [], vibeZ: [], accX: [], accY: [], accZ: [], gyrX: [], gyrY: [], gyrZ: [], gpsAlt: [], baroAlt: [], arspd: [], gpsErrH: [], gpsSats: [], hdop: [] };
  const streams: Stream[] = [];
  const S = (name: string, format: string, labels: string): Stream => {
    const s = { name, format, labels: labels.split(','), rows: [] as (number | string)[][] };
    streams.push(s);
    return s;
  };

  // ---------------- parameters / mode / messages ----------------
  const PARM = S('PARM', 'QNff', 'TimeUS,Name,Value,Default');
  const f = product.fdm;
  const params: [string, number][] = [
    ['Q_ENABLE', 1], ['Q_TILT_ENABLE', 1], ['Q_TILT_TYPE', 0], ['Q_TILT_MASK', 15],
    ['Q_TILT_RATE_UP', f?.hingeActuator.rateDegS ?? 25], ['Q_TILT_RATE_DN', f?.hingeActuator.rateDegS ?? 25],
    ['Q_ASSIST_SPEED', f?.transition.assistSpeedMs ?? 0], ['ARSPD_FBW_MIN', f?.transition.fbwMinSpeedMs ?? 0], ['TRIM_ARSPD_CM', Math.round(trace.config.cruiseSpeedMs * 100)],
    ['BATT_CAPACITY', bat ? bat.cellCapacityAh * bat.cellsP * 1000 : 0], ['BATT_LOW_VOLT', bat ? bat.lowCellV * bat.cellsS : 0], ['BATT_CRT_VOLT', bat ? bat.criticalCellV * bat.cellsS : 0],
    ['BATT_ARM_VOLT', bat ? bat.armCellV * bat.cellsS : 0], ['Q_WVANE_ENABLE', 1], ['SIM_SPECTER', 1],
  ];
  for (const [k, v] of params) PARM.rows.push([0, k, v, v]);
  const MSG = S('MSG', 'QZ', 'TimeUS,Message');
  MSG.rows.push([0, `Specter Configurator simulated log - ${ascii(product.name)} - ESTIMATE, NOT FLIGHT DATA`]);
  for (const e of trace.events) MSG.rows.push([us(e.t), ascii(`${e.level === 'info' ? '' : e.level.toUpperCase() + ': '}${e.text}`).slice(0, 63)]);
  const MODE = S('MODE', 'QMBB', 'TimeUS,Mode,ModeNum,Rsn');
  MODE.rows.push([0, AP_MODE.AUTO, AP_MODE.AUTO, 1]);

  // ---------------- per-sample streams at the trace rate ----------------
  const ATT = S('ATT', 'QccccCCCCB', 'TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw,AEKF');
  const POS = S('POS', 'QLLfff', 'TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt');
  const GPS = S('GPS', 'QBBIHBcLLeffffB', 'TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U');
  const BARO = S('BARO', 'QBfffcfIffB', 'TimeUS,I,Alt,AltAMSL,Press,Temp,CRt,SMS,Offset,GndTemp,Health');
  const ARSP = S('ARSP', 'QBffcffBBffB', 'TimeUS,I,Airspeed,DiffPress,Temp,RawPress,Offset,U,H,Hp,TR,Pri');
  const BAT = S('BAT', 'QBfffffcfBBB', 'TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H,SH');
  const ESC = S('ESC', 'QBffffcfcf', 'TimeUS,Instance,RPM,RawRPM,Volt,Curr,Temp,CTot,MotTemp,Err');
  const RCOU = S('RCOU', 'QHHHHHHHHHHHHHH', 'TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14');
  const CTUN = S('CTUN', 'Qcccchhhfff', 'TimeUS,NavRoll,Roll,NavPitch,Pitch,ThO,RdO,ThD,As,SAs,E2T');
  const QTUN = S('QTUN', 'QffffffffB', 'TimeUS,ThO,ThH,DAlt,Alt,DCRt,CRt,TMix,Tilt,Trn');
  const NTUN = S('NTUN', 'Qfccfff', 'TimeUS,Dist,TBrg,NavBrg,AltE,XT,AsE');
  const VIBE = S('VIBE', 'QBfffI', 'TimeUS,IMU,VibeX,VibeY,VibeZ,Clip');
  const HNG = S('HNG', 'QBffff', 'TimeUS,I,Ang,DAng,Trq,Lim');
  const LOAD = S('LOAD', 'Qfffff', 'TimeUS,Nz,BendL,BendR,AoA,Stall');
  const WND = S('WND', 'Qfff', 'TimeUS,N,E,U');
  const PHS = S('PHS', 'QBN', 'TimeUS,Id,Name');

  // slowly varying sensor errors (first-order Gauss–Markov)
  const gm = (sigma: number, tau: number, dt: number, x: number) => {
    const p = Math.exp(-dt / tau);
    return p * x + sigma * Math.sqrt(1 - p * p) * rnd();
  };
  let gpsN = sp.gpsHorizSigmaM * rnd(), gpsE = sp.gpsHorizSigmaM * rnd(), gpsU = sp.gpsVertSigmaM * rnd(), baroD = sp.baroDriftM * rnd();
  let sats = 16;
  const gpsWeek0 = Math.floor((opt.startEpochMs - Date.UTC(1980, 0, 6)) / (7 * 86400e3));
  const gpsMs0 = (opt.startEpochMs - Date.UTC(1980, 0, 6)) % (7 * 86400e3);
  const cosLat = Math.cos(opt.homeLatDeg * DEG);
  const hoverThr = (() => {
    // mean hover throttle (for QTUN.ThH): average motor duty during the first hover phase
    let s = 0, k = 0;
    for (let i = 0; i < n; i++) if (c.phase[i] === 1) { for (let r = 0; r < nR; r++) s += c[`thr${r + 1}`][i]; k += nR; }
    return k ? s / k / 100 : 0.5;
  })();
  let lastPhase = -1;
  let prevAlt = c.alt[0] ?? 0;
  const gpsEvery = Math.max(1, Math.round(0.2 / dtTrace));

  for (let i = 0; i < n; i++) {
    const t = c.t[i], T = us(t);
    const ph = c.phase[i];
    if (ph !== lastPhase) {
      PHS.rows.push([T, ph, ascii(PHASES[ph] ?? '')]);
      lastPhase = ph;
    }
    const vtol = ph <= 2 || ph >= 6;
    // --- attitude
    ATT.rows.push([T, c.rollCmd[i], c.roll[i], c.pitchCmd[i], c.pitch[i], c.yaw[i], c.yaw[i], Math.hypot(c.rollCmd[i] - c.roll[i], c.pitchCmd[i] - c.pitch[i]) / 100, 0, 3]);
    // --- position sensors
    gpsN = gm(sp.gpsHorizSigmaM, 30, dtTrace, gpsN); gpsE = gm(sp.gpsHorizSigmaM, 30, dtTrace, gpsE); gpsU = gm(sp.gpsVertSigmaM, 45, dtTrace, gpsU);
    baroD = gm(sp.baroDriftM, 90, dtTrace, baroD);
    const lat = opt.homeLatDeg + (c.pn[i] / R_EARTH) / DEG, lon = opt.homeLonDeg + (c.pe[i] / (R_EARTH * cosLat)) / DEG;
    POS.rows.push([T, lat, lon, opt.homeAltMslM + c.alt[i], c.alt[i], c.alt[i]]);
    const gAlt = opt.homeAltMslM + c.alt[i] + gpsU;
    if (i % gpsEvery === 0) {
      if (rnd() > 1.6) sats = Math.max(11, Math.min(20, sats + (rnd() > 0 ? 1 : -1)));
      const glat = opt.homeLatDeg + ((c.pn[i] + gpsN) / R_EARTH) / DEG, glon = opt.homeLonDeg + ((c.pe[i] + gpsE) / (R_EARTH * cosLat)) / DEG;
      const gms = gpsMs0 + t * 1000;
      const vn = c.vn[i] + sp.gpsVelSigmaMs * rnd(), ve = c.ve[i] + sp.gpsVelSigmaMs * rnd();
      GPS.rows.push([T, 0, 3, Math.round(gms % (7 * 86400e3)), gpsWeek0 + Math.floor(gms / (7 * 86400e3)), sats, 0.6 + 4 / sats, glat, glon, gAlt, Math.hypot(vn, ve), ((Math.atan2(ve, vn) / DEG) + 360) % 360, c.vd[i] + sp.gpsVelSigmaMs * rnd(), 0, 1]);
    }
    const rho = c.rho[i] ?? 1.225;
    const bAlt = c.alt[i] + baroD + sp.baroNoiseM * rnd();
    const tempC = 15 + trace.config.env.isaDeltaK - 0.0065 * (opt.homeAltMslM + c.alt[i]);
    const press = rho * 287.05 * (tempC + 273.15);
    BARO.rows.push([T, 0, bAlt, opt.homeAltMslM + bAlt, press, tempC, (c.alt[i] - prevAlt) / dtTrace, Math.round(t * 1000), 0, 15 + trace.config.env.isaDeltaK, 1]);
    prevAlt = c.alt[i];
    // pitot sees the forward component only; noisy and unreliable at very low speed
    const vTrue = Math.max(0, c.ias[i] * Math.cos((c.aoa[i] ?? 0) * DEG) * Math.cos((c.beta[i] ?? 0) * DEG));
    const as = Math.max(0, (vtol && c.ias[i] < 6 ? 0.4 * c.ias[i] : vTrue) + sp.airspeedNoiseMs * rnd() * (c.ias[i] < 8 ? 2 : 1));
    ARSP.rows.push([T, 0, as, 0.5 * rho * as * as, tempC, 0.5 * rho * as * as, 0, 1, 1, 1, 1, 1]);
    // --- power system
    const cap = bat ? bat.cellCapacityAh * bat.cellsP : 1;
    BAT.rows.push([T, 0, c.battV[i] + 0.02 * rnd(), c.battV[i] + (c.battI[i] * (bat ? (bat.cellResistanceOhm * bat.cellsS) / bat.cellsP : 0)), c.battI[i] * (1 + 0.005 * rnd()), c.usedAh[i] * 1000, c.energyWh[i], c.cellT[i], bat ? (bat.cellResistanceOhm * bat.cellsS) / bat.cellsP : 0, Math.max(0, Math.min(100, Math.round(c.soc[i]))), 1, 1]);
    void cap;
    const pwm: number[] = [];
    for (let k = 0; k < nR; k++) {
      const rpm = c[`rpm${k + 1}`][i], thr = c[`thr${k + 1}`][i];
      ESC.rows.push([T, k, rpm, rpm * (1 + 0.002 * rnd()), c.battV[i], (c[`cur${k + 1}`][i] * thr) / 100, c[`tesc${k + 1}`][i], 0, c[`tmot${k + 1}`][i], 0]);
      pwm.push(Math.round(1000 + 10 * thr));
    }
    while (pwm.length < 4) pwm.push(1000);
    const srv = (deg: number, lim: number) => Math.round(1500 + 400 * Math.max(-1, Math.min(1, deg / lim)));
    // C5 aileron, C6/C7 left/right ruddervator, C8 wing fold (tilt) actuator
    RCOU.rows.push([T, pwm[0], pwm[1], pwm[2], pwm[3], srv(c.ail[i], 25), srv(-c.elev[i] - c.rud[i], 25), srv(-c.elev[i] + c.rud[i], 25), Math.round(1000 + 1000 * (1 - (c.fold[i] ?? 0) / maxFold)), 1500, 1500, 1500, 1500, 1500, 1500]);
    const thrMean = Array.from({ length: nR }, (_, k) => c[`thr${k + 1}`][i]).reduce((a, b) => a + b, 0) / nR;
    // --- controller tuning messages
    if (!vtol || ph === 2 || ph === 6) CTUN.rows.push([T, c.rollCmd[i], c.roll[i], c.pitchCmd[i], c.pitch[i], Math.round(thrMean), Math.round((c.rud[i] / 25) * 4500), Math.round(thrMean), as, c.ias[i], c.ias[i] > 3 ? c.gs[i] / c.ias[i] : 1]);
    if (vtol) QTUN.rows.push([T, thrMean / 100, hoverThr, c.altCmd[i], c.alt[i], 0, -c.vd[i], Math.min(1, Math.max(0, (c.fold[i] ?? 0) / maxFold)), 90 * (1 - (c.fold[i] ?? 0) / maxFold), ph === 2 ? 1 : ph === 6 ? 2 : 0]);
    if (!vtol) NTUN.rows.push([T, c.wpDist[i], 0, c.yaw[i], c.altCmd[i] - c.alt[i], c.xtrack[i], c.spdCmd[i] - c.ias[i]]);
    // --- vibration (RMS, as ArduPilot VIBE)
    const vr = vibRms(i).map((v) => v * (1 + 0.05 * rnd()));
    VIBE.rows.push([T, 0, vr[0], vr[1], vr[2], 0]);
    ch.vibeX.push(vr[0]); ch.vibeY.push(vr[1]); ch.vibeZ.push(vr[2]);
    // --- custom Specter messages: hinge and structural loads, wind truth
    const lim = f?.hingeActuator.maxTorqueNm ?? 0;
    HNG.rows.push([T, 0, c.fold[i], c.foldCmd[i], c.hingeL[i], lim]);
    HNG.rows.push([T, 1, c.fold[i], c.foldCmd[i], c.hingeR[i], lim]);
    LOAD.rows.push([T, c.nz[i], c.bendL[i], c.bendR[i], c.aoa[i], c.stall[i]]);
    WND.rows.push([T, c.windN[i], c.windE[i], c.windU[i]]);
    ch.gpsAlt.push(gAlt - opt.homeAltMslM); ch.baroAlt.push(bAlt); ch.arspd.push(as); ch.gpsErrH.push(Math.hypot(gpsN, gpsE)); ch.gpsSats.push(sats); ch.hdop.push(0.6 + 4 / sats);
    // IMU value at the trace rate for the charts (noise + vibration RMS-equivalent sample)
    ch.accX.push(c.fx[i] + accBias[0] + vr[0] * rnd()); ch.accY.push(c.fz[i] + accBias[1] + vr[1] * rnd()); ch.accZ.push(-c.fy[i] + accBias[2] + vr[2] * rnd());
    ch.gyrX.push(c.p[i] * DEG + gyrBias[0] + sp.gyroNoiseRadS * rnd()); ch.gyrY.push(c.q[i] * DEG + gyrBias[1] + sp.gyroNoiseRadS * rnd()); ch.gyrZ.push(c.r[i] * DEG + gyrBias[2] + sp.gyroNoiseRadS * rnd());
  }

  // ---------------- IMU at its own rate (linear interpolation of truth + vibration tones) ----------------
  const IMU = S('IMU', 'QBffffffIIfBBHH', 'TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz');
  const tEnd = c.t[n - 1] ?? 0;
  const dtI = 1 / opt.imuRateHz;
  const lerp = (a: number[], i: number, u: number) => a[i] + ((a[Math.min(n - 1, i + 1)] ?? a[i]) - a[i]) * u;
  for (let k = 0; k * dtI <= tEnd; k++) {
    const t = k * dtI;
    const fi = t / dtTrace;
    const i = Math.min(n - 1, Math.floor(fi));
    const u = Math.min(1, fi - i);
    const v = vibAmp(i);
    let vx = 0, vy = 0, vz = 0;
    for (let r = 0; r < nR; r++) {
      const w = 2 * Math.PI * v.freq[r] * t;
      const s1 = Math.sin(w + phase1[r]), c1 = Math.cos(w + phase1[r]), s2 = Math.sin(2 * w + phase2[r]);
      // rotating imbalance force in the rotor plane, blade-pass tone along the thrust axis
      vx += v.f * v.inPlane[r] * c1 + (1 - v.f) * v.axial[r] * s2;
      vy += v.inPlane[r] * s1;
      vz += v.f * v.axial[r] * s2 + (1 - v.f) * v.inPlane[r] * c1;
    }
    const b = v.buffet / Math.sqrt(3);
    IMU.rows.push([
      us(t), 0,
      lerp(c.p, i, u) * DEG + gyrBias[0] + sp.gyroNoiseRadS * rnd(), lerp(c.q, i, u) * DEG + gyrBias[1] + sp.gyroNoiseRadS * rnd(), lerp(c.r, i, u) * DEG + gyrBias[2] + sp.gyroNoiseRadS * rnd(),
      lerp(c.fx, i, u) + accBias[0] + vx + (sp.accelNoiseMs2 + b) * rnd(), lerp(c.fz, i, u) + accBias[1] + vy + (sp.accelNoiseMs2 + b) * rnd(), -lerp(c.fy, i, u) + accBias[2] + vz + (sp.accelNoiseMs2 + b) * rnd(),
      0, 0, 35 + 0.002 * t, 1, 1, opt.imuRateHz, opt.imuRateHz,
    ]);
  }

  return {
    productId: trace.productId, options: opt, streams, ch,
    assumptions: [
      'Telemetri, 6-DOF simülasyonun “gerçek” kanallarından türetilmiş TAHMİNİ veridir; uçuş verisi değildir (logda MSG ve SIM_SPECTER parametresi ile işaretlenir).',
      'IMU: gerçek özgül kuvvet / açısal hız + sabit bias + beyaz gürültü + rotor titreşim tonları (1/dev dengesizlik rotor düzleminde ∝ rpm², kanat geçiş tonu itki ekseninde ∝ itki) + stall/katlanma buffet’i.',
      'VIBE: ArduPilot’taki gibi eksen başına ivme RMS değeri; titreşim genlikleri varsayımdır (yumuşak montajlı Cube sınıfı otopilot) — ilk uçuş logu ile kalibre edilmelidir.',
      'GPS: yatay σ 0,8 m / düşey σ 1,5 m Gauss–Markov hata, 5 Hz; baro: 0,5 m sürüklenme + gürültü; pitot: ileri hız bileşeni + gürültü, düşük hızda güvenilmez.',
      'ESC akımı batarya tarafı akımıdır (faz akımı × görev oranı). RCOU: C1–C4 motorlar, C5 aileron, C6/C7 ruddervator, C8 kanat katlama eyleyicisi.',
      'HNG (menteşe açısı / momenti), LOAD (yük faktörü, kanat kökü eğilme momenti), WND ve PHS Specter’e özgü ek mesajlardır.',
      'Uçuş modu tüm görev boyunca AUTO olarak loglanır; faz geçişleri MSG ve PHS mesajlarındadır.',
    ],
  };
}
