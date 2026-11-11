/**
 * Full-mission 6-DOF simulation: vertical take-off → hover → forward transition → climb / waypoint
 * cruise → descent → back-transition → hover → vertical landing.
 *
 * Integration: RK4, dt = 0.01 s on [position, velocity, attitude quaternion, body rates, rotor speeds].
 * Controller (100 Hz, ArduPilot-QuadPlane-like structure, simplified):
 *   VTOL: position → velocity → tilt, altitude → total thrust, attitude → rate → moment;
 *   forward transition: wing opening scheduled on airspeed (fully open at openCompleteSpeedMs);
 *   fixed wing: L1 lateral guidance, altitude by pitch, airspeed by thrust, coordinated turns;
 *   control allocation: weighted pseudo-inverse over 4 rotor thrusts + aileron / elevator / rudder
 *   (differential thrust is blended out as airspeed rises).
 */
import type { PayloadOption, Product } from '../../data/schema';
import { atmosphere } from '../atmosphere';
import { solve } from '../rotor';
import type { Environment } from '../types';
import { drydenParams, gaussian, oneMinusCosineGust, windAtHeight } from '../wind';
import { dutyForOmega, evalForces, newForceOutput, omegaForThrust, propPoint, type ForceOutput } from './forces';
import { DEG, G, GeomState, interp, mulMV } from './model';
import { activeMask, MODEL_ASSUMPTIONS, panelLoads, prepareModel, trimLevel } from './trim';

export interface MissionConfig {
  hoverHeightM: number;
  cruiseAltitudeM: number;
  cruiseSpeedMs: number;
  takeoffHoverS: number;
  landingHoverS: number;
  /** [north, east] metres from the launch point; the aircraft returns to launch after the last one */
  waypointsM: [number, number][];
  cruiseMotors: 2 | 4;
  env: Environment;
  seed: number;
  gustStartS: number;
  gustDurationS: number;
  initialSoc: number;
  outputHz?: number;
  maxDurationS?: number;
  /** distance from the landing point at which the back-transition starts */
  backTransitionDistanceM?: number;
  /** pre-calibrated parasite drag area (skips the calibration) */
  cdaFront?: number;
}

export const PHASES = ['Kalkış', 'Hover (çıkış)', 'İleri geçiş', 'Tırmanış', 'Seyir', 'Alçalma', 'Geri geçiş', 'Hover (dönüş)', 'İniş', 'Bitti'] as const;
export const enum Ph { TAKEOFF, HOVER_OUT, TRANSITION, CLIMB, CRUISE, DESCENT, BACK, HOVER_IN, LAND, DONE }

export interface MissionEvent {
  t: number;
  level: 'info' | 'warn' | 'alarm';
  text: string;
}

export interface PhaseSummary {
  phase: number;
  name: string;
  tStart: number;
  tEnd: number;
  energyWh: number;
  distanceM: number;
  meanPowerW: number;
  peakPowerW: number;
}

export interface MissionSummary {
  completed: boolean;
  outcome: string;
  durationS: number;
  distanceM: number;
  energyWh: number;
  finalSoc: number;
  minCellV: number;
  maxBatteryCurrentA: number;
  maxCRate: number;
  timeAboveContinuousCS: number;
  maxMotorTempC: number;
  maxEscTempC: number;
  maxHingeMomentNm: number;
  maxLoadFactor: number;
  maxBankDeg: number;
  maxAltErrorM: number;
  maxCrossTrackM: number;
  transitionAltLossM: number;
  transitionDurationS: number;
  backTransitionDurationS: number;
  saturationTimeS: number;
  stallTimeS: number;
  assistTimeS: number;
  touchdownSinkMs: number;
  hoverTimeS: number;
  cruiseMeanPowerW: number;
  hoverMeanPowerW: number;
}

export interface MissionTrace {
  productId: string;
  config: MissionConfig;
  /** telemetry channels, all sampled at outputHz */
  ch: Record<string, number[]>;
  nRotors: number;
  phases: PhaseSummary[];
  events: MissionEvent[];
  summary: MissionSummary;
  assumptions: string[];
  cdaFront: number;
  wallTimeMs: number;
}

const clamp = (x: number, a: number, b: number) => Math.min(b, Math.max(a, x));
const wrapPi = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

function quatToMat(q: number[], R: number[]) {
  const [w, x, y, z] = q;
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
}

export const MISSION_ASSUMPTIONS: string[] = [
  'Entegrasyon RK4, dt = 0,01 s; kontrolcü 100 Hz, adım boyunca sabit (ZOH). Yer teması: kalkışa kadar sabit, inişte temas anında durur.',
  'Kontrolcü durumları doğrudan kullanır (ideal kestirim); sensör gürültüsü ve EKF gecikmeleri Faz 2 telemetri modelinde eklenecek.',
  'Kontrol dağıtımı: model tabanlı ağırlıklı sözde-ters (4 rotor itkisi + aileron/elevator/rudder); diferansiyel itki ağırlığı hava hızıyla azaltılır.',
  'İleri geçiş: kanat açılma oranı = max(başlangıç oranı, V/V_açık); V_açık hızında tam açık. Başlangıç oranı (hızlanabilmek için) varsayımdır. Geçiş koruması: hız 0,85·V_açık altındayken uçak alçalıyorsa veya geçiş irtifasının 1,2 m altındaysa açılma bekletilir.',
  'Geri geçiş (varsayım): önce sabit kanat konumunda rölanti itkiyle ≈1,07·Vs hızına yavaşlanır; sonra kanat eyleyici hızıyla katlanır (katlanan panel hücum açısı artar ve hava freni gibi çalışır), itki ekseni yukarı döndükçe itki devreye girer, son yavaşlama hover kontrolcüsüyle yapılır. Yaklaşma ve geri geçiş rüzgâra karşı uçulur.',
  'Sabit kanat: L1 yanal güdüm (periyot 17 s, sönüm 0,75), irtifa → yunuslama, hava hızı → itki, koordineli dönüş; yatış sınırı stall marjına göre (en çok 35°). Düşük hızda hız önceliği (burun aşağı), hücum açısı sınırı ve stall kurtarma (kanadı boşalt + güç) vardır.',
  'İtki → devir → ESC görev oranı ileri beslemesi pervane/motor modelinden (itki doğrusallaştırma); motor tepkisi elektromekanik zaman sabitiyle sınırlı.',
  'Batarya: hücre OCV(SoC) tablosu + seri direnç + tek RC kolu (R1 = 0,4·R0, τ = 25 s); sıcaklık etkisi yok. Aviyonik + payload tüketimi sabit 60 W.',
  'Motor / ESC sıcaklığı tek düğümlü ısıl model (bakır + demir kaybı, hava akışıyla artan soğutma) — kaba tahmin.',
  'Rüzgâr: 10 m referanslı logaritmik profil + seed’li Dryden türbülansı (irtifaya göre σ, L) + ortalama rüzgâr yönünde 1-cosine gust.',
  'Rüzgâr yönü pusula yönüdür (geldiği yön). Rüzgâr > 1,5 m/s ise kalkış ve ileri geçiş burun rüzgâra dönük yapılır; hover’da rüzgâra burun çevirme (weathervane) etkindir.',
  'İniş hover süresi, alçalma dahil toplam hover (dönüş) + iniş süresidir; kalkış hover süresi tırmanma dahildir.',
];

export function simulateMission(product: Product, payload: PayloadOption | null, cfg: MissionConfig, onProgress?: (frac: number, phase: number) => void): MissionTrace {
  const wall0 = Date.now();
  const { m, rho: rho0, vNom } = prepareModel(product, payload, cfg.env.altitudeM, cfg.env.isaDeltaK, cfg.cdaFront);
  const spec = m.spec;
  const geom = new GeomState(m);
  const nR = m.rotors.length;
  const dt = 0.01;
  const outEvery = Math.max(1, Math.round(1 / ((cfg.outputHz ?? 10) * dt)));
  const mass = m.massKg;
  const W = mass * G;
  const bat = spec.battery;
  const capAh = bat.cellCapacityAh * bat.cellsP;
  const R0 = (bat.cellResistanceOhm * bat.cellsS) / bat.cellsP;
  const tr = spec.transition;
  const vOpen = tr.openCompleteSpeedMs;
  const maxFold = m.maxFoldRad;
  const foldRate = spec.hingeActuator.rateDegS * DEG;
  const surfMax = [(product.geometry.aileron?.maxDeflectionDeg ?? 25) * DEG, 20 * DEG, 20 * DEG];
  const D = m.rotors[0].diameter;

  // ---- trim tables for the fixed-wing controller (α and thrust vs airspeed)
  const trimV: number[] = [], trimA: number[] = [], trimT: number[] = [];
  for (let v = Math.max(12, vOpen * 0.85); v <= cfg.cruiseSpeedMs * 1.5 + 6; v += 1.5) {
    const t = trimLevel(m, geom, v, rho0, cfg.cruiseMotors, vNom);
    if (t.converged) {
      trimV.push(v); trimA.push(t.x[0]); trimT.push(t.thrustN);
    }
  }
  if (trimV.length < 2) throw new Error('Sabit kanat trim tablosu oluşturulamadı (seyir hızı stall hızına çok yakın olabilir).');
  // max thrust per rotor vs axial speed (current / voltage limited)
  const tmV: number[] = [], tmT: number[] = [];
  for (let v = 0; v <= 60; v += 5) {
    let lo = 0, hi = 1500;
    for (let i = 0; i < 30; i++) {
      const om = (lo + hi) / 2;
      const pp = propPoint(m, om, v, rho0, D);
      const iNeed = Math.max(0, pp.P / Math.max(1, om) / m.kt) + spec.motor.noLoadCurrentA;
      if (iNeed <= spec.motor.maxCurrentA * 0.97 && om / m.kvRad + iNeed * m.rTot <= vNom * 0.93) lo = om;
      else hi = om;
    }
    tmV.push(v); tmT.push(Math.max(0, propPoint(m, lo, v, rho0, D).T));
  }
  const kQ = (spec.prop.cp[0] * D) / (2 * Math.PI * spec.prop.ct[0]);
  // fixed-wing sink rate limit: stay inside the power-off glide (otherwise the aircraft overspeeds)
  const ldMax = Math.max(...trimT.map((T) => W / Math.max(1e-6, T)));
  const fwSink = Math.min(2.5, (0.8 * cfg.cruiseSpeedMs) / ldMax);
  // speed at which the back-transition starts folding (just above the wing-open speed / stall)
  const vDecel = Math.max(vOpen + 0.8, 1.07 * Math.sqrt((2 * W) / (rho0 * m.Sref * product.aero.clMax)));
  // ESC thermal node sized so that the continuous current rating gives ≈ 55 K rise in still air
  const iEsc = spec.esc.maxContinuousCurrentA;
  const escRth = 55 / (iEsc * iEsc * spec.esc.resistanceOhm + 0.012 * iEsc * 0.4 * vNom);
  const escC = 0.8 * iEsc;
  const vStall = Math.sqrt((2 * W) / (rho0 * m.Sref * product.aero.clMax));
  // body angle of attack at which the wing reaches ≈ 85 % of the section stall angle
  const alphaMax = 0.85 * (spec.section.alphaZeroLiftDeg * DEG + spec.section.clMax / (m.vlm.clAlphaPerRad * (spec.section.liftSlopePerRad / (2 * Math.PI)))) - product.geometry.incidenceDeg * DEG;

  // ---- route
  const home: [number, number] = [0, 0];
  const dBack = cfg.backTransitionDistanceM ?? 250;
  const route: [number, number][] = [home, ...cfg.waypointsM];
  if (cfg.env.windSpeedMs > 1.5) {
    // approach point downwind of the landing site: the final leg and the back-transition are flown into the wind
    const dA = dBack + 450;
    route.push([Math.cos(cfg.env.windFromDeg * DEG) * -dA, Math.sin(cfg.env.windFromDeg * DEG) * -dA]);
  }
  route.push(home);
  const legBearing = Math.atan2(route[1][1] - route[0][1], route[1][0] - route[0][0]);
  const approachAlt = cfg.hoverHeightM + 30;

  // ---- state
  // x = [p(3), v(3), q(4), w(3), Ω(nR)]
  const NX = 13 + nR;
  let x = new Array<number>(NX).fill(0);
  // initial heading: nose into the wind when there is wind (standard VTOL practice), else along the first leg
  const psi0 = cfg.env.windSpeedMs > 1.5 ? wrapPi(cfg.env.windFromDeg * DEG) : legBearing;
  x[6] = Math.cos(psi0 / 2); // rotation about −y for yaw right
  x[8] = -Math.sin(psi0 / 2);
  let fold = maxFold;
  const surf = [0, 0, 0];
  const surfCmd = [0, 0, 0];
  const duty = new Array<number>(nR).fill(0);
  let onGround = true;
  let usedAh = 0, vRc = 0, vBatt = bat.cellsS * interp(bat.ocvSoc, bat.ocvV, cfg.initialSoc);
  const tMotor = new Array<number>(nR).fill(0), tEsc = new Array<number>(nR).fill(0);
  const ambient = 15 + cfg.env.isaDeltaK;
  for (let k = 0; k < nR; k++) { tMotor[k] = ambient; tEsc[k] = ambient; }
  let cellTemp = ambient;

  // ---- wind
  const windTo = [-Math.cos(cfg.env.windFromDeg * DEG), 0, -Math.sin(cfg.env.windFromDeg * DEG)];
  const windCross = [-windTo[2], 0, windTo[0]];
  const rand = gaussian(cfg.seed);
  const turb = [0, 0, 0];
  let dry = drydenParams(cfg.env.turbulence, 10).value;
  const wind = [0, 0, 0];
  let rho = rho0;

  // ---- controller state
  let phase: number = Ph.TAKEOFF;
  let phaseT0 = 0;
  let leg = 0;
  let iVy = 0, iP = 0, iQ = 0, iR = 0, iAlt = 0, iSpd = 0;
  let hRef = 0;
  const faeroW = [0, 0, 0];
  const maeroB = [0, 0, 0];
  const Bsurf = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]; // dM/dδ columns
  let phiCmdPrev = 0;
  let transStartT = 0, transStartAlt = 0, transMinAlt = 0, backStartT = 0;
  let headingCmd = psi0;
  let backFold = false;
  let openPrev = 0;
  let fwEntryT = 0;
  // forward transition is flown into the wind when there is wind (the folded panels inside the
  // propeller slipstream cannot take a crosswind); L1 then joins the first leg in fixed-wing flight
  const trBearing = cfg.env.windSpeedMs > 1.5 ? psi0 : legBearing;
  const hoverPos: [number, number] = [0, 0];
  let prevWingCl = 0;
  const out1 = newForceOutput(nR), outS = newForceOutput(nR), outP = newForceOutput(nR);
  const R = new Array<number>(9).fill(0);

  // ---- logging
  const names = [
    't', 'phase', 'pn', 'pe', 'alt', 'vn', 've', 'vd', 'roll', 'pitch', 'yaw', 'p', 'q', 'r', 'ias', 'gs', 'aoa', 'beta', 'fold', 'foldCmd',
    'altCmd', 'spdCmd', 'rollCmd', 'pitchCmd', 'ail', 'elev', 'rud', 'battV', 'battI', 'soc', 'power', 'cellV', 'cellT', 'cRate',
    'thrustTot', 'hingeL', 'hingeR', 'nz', 'windN', 'windE', 'windU', 'xtrack', 'wpDist', 'stall', 'sat', 'cl', 'energyWh',
    'fx', 'fy', 'fz', 'bendL', 'bendR', 'rho', 'usedAh', 'wp',
  ];
  for (let k = 0; k < nR; k++) names.push(`rpm${k + 1}`, `thr${k + 1}`, `thrust${k + 1}`, `cur${k + 1}`, `tmot${k + 1}`, `tesc${k + 1}`);
  const ch: Record<string, number[]> = {};
  for (const n of names) ch[n] = [];
  const events: MissionEvent[] = [];
  const phases: PhaseSummary[] = [];
  const ev = (t: number, level: MissionEvent['level'], text: string) => events.push({ t, level, text });
  const flags: Record<string, boolean> = {};
  const once = (key: string, t: number, level: MissionEvent['level'], text: string) => {
    if (!flags[key]) {
      flags[key] = true;
      ev(t, level, text);
    }
  };

  // ---- summary accumulators
  let energyJ = 0, dist = 0, minCellV = 9, maxI = 0, tAboveC = 0, maxTm = ambient, maxTe = ambient, maxHinge = 0, maxNz = 1, maxBank = 0;
  let maxAltErr = 0, maxXt = 0, satTime = 0, stallTime = 0, assistTime = 0, touchdown = 0, hoverTime = 0;
  let phE = 0, phD = 0, phPk = 0;
  let cruiseE = 0, cruiseT = 0, hoverE = 0;
  let outcome = '';
  let completed = false;
  const setPhase = (p: number, t: number) => {
    phases.push({ phase, name: PHASES[phase], tStart: phaseT0, tEnd: t, energyWh: phE / 3600, distanceM: phD, meanPowerW: t > phaseT0 ? phE / (t - phaseT0) : 0, peakPowerW: phPk });
    phE = 0; phD = 0; phPk = 0;
    phase = p;
    phaseT0 = t;
    ev(t, 'info', `Faz: ${PHASES[p]}`);
  };
  ev(0, 'info', `Faz: ${PHASES[phase]}`);

  // ---- derivative
  const inp = { vb: [0, 0, 0], w: [0, 0, 0], omega: new Array<number>(nR).fill(0), duty, surf, rho, vBatt };
  const deriv = (s: number[], o: ForceOutput): number[] => {
    const q = [s[6], s[7], s[8], s[9]];
    const qn = Math.hypot(q[0], q[1], q[2], q[3]);
    for (let i = 0; i < 4; i++) q[i] /= qn;
    quatToMat(q, R);
    const ax = s[3] - wind[0], ay = s[4] - wind[1], az = s[5] - wind[2];
    inp.vb[0] = R[0] * ax + R[3] * ay + R[6] * az;
    inp.vb[1] = R[1] * ax + R[4] * ay + R[7] * az;
    inp.vb[2] = R[2] * ax + R[5] * ay + R[8] * az;
    inp.w[0] = s[10]; inp.w[1] = s[11]; inp.w[2] = s[12];
    for (let k = 0; k < nR; k++) inp.omega[k] = Math.max(0, s[13 + k]);
    inp.rho = rho; inp.vBatt = vBatt;
    evalForces(m, geom, inp, o, prevWingCl);
    const F = o.F, M = o.M, w = inp.w;
    const d = new Array<number>(NX);
    d[0] = s[3]; d[1] = s[4]; d[2] = s[5];
    d[3] = (R[0] * F[0] + R[1] * F[1] + R[2] * F[2]) / mass;
    d[4] = (R[3] * F[0] + R[4] * F[1] + R[5] * F[2]) / mass - G;
    d[5] = (R[6] * F[0] + R[7] * F[1] + R[8] * F[2]) / mass;
    d[6] = 0.5 * (-q[1] * w[0] - q[2] * w[1] - q[3] * w[2]);
    d[7] = 0.5 * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]);
    d[8] = 0.5 * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0]);
    d[9] = 0.5 * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0]);
    const I = geom.I;
    const hx = I[0] * w[0] + I[1] * w[1] + I[2] * w[2] + o.hRot[0];
    const hy = I[3] * w[0] + I[4] * w[1] + I[5] * w[2] + o.hRot[1];
    const hz = I[6] * w[0] + I[7] * w[1] + I[8] * w[2] + o.hRot[2];
    const tx = M[0] - (w[1] * hz - w[2] * hy), ty = M[1] - (w[2] * hx - w[0] * hz), tz = M[2] - (w[0] * hy - w[1] * hx);
    const Ii = geom.Iinv;
    d[10] = Ii[0] * tx + Ii[1] * ty + Ii[2] * tz;
    d[11] = Ii[3] * tx + Ii[4] * ty + Ii[5] * tz;
    d[12] = Ii[6] * tx + Ii[7] * ty + Ii[8] * tz;
    for (let k = 0; k < nR; k++) d[13 + k] = o.omegaDot[k];
    return d;
  };

  const maxT = cfg.maxDurationS ?? 4 * 3600;
  const nSteps = Math.round(maxT / dt);
  let estTotal = 600;
  {
    let d = 0;
    for (let i = 0; i < route.length - 1; i++) d += Math.hypot(route[i + 1][0] - route[i][0], route[i + 1][1] - route[i][1]);
    estTotal = cfg.takeoffHoverS + cfg.landingHoverS + d / cfg.cruiseSpeedMs + 120;
  }
  let doneAt = -1;
  let foldCmd = maxFold;
  let altCmd = 0, spdCmd = 0, rollCmd = 0, pitchCmd = 0, xtrack = 0, wpDist = 0, satNow = 0;
  let tDesc = -1;

  for (let step = 0; step <= nSteps; step++) {
    const t = step * dt;
    // ================= environment =================
    const alt = x[1];
    if (step % 100 === 0) {
      rho = atmosphere(cfg.env.altitudeM + alt, cfg.env.isaDeltaK).value.densityKgM3;
      dry = drydenParams(cfg.env.turbulence, Math.max(3, alt)).value;
    }
    const meanW = windAtHeight(cfg.env.windSpeedMs, Math.max(1, alt), cfg.env.logProfile);
    const vAirNow = Math.hypot(x[3] - wind[0], x[4] - wind[1], x[5] - wind[2]);
    if (cfg.env.turbulence !== 'none') {
      const conv = Math.max(vAirNow, meanW, 1);
      for (let i = 0; i < 3; i++) {
        const phi = Math.exp((-conv * dt) / Math.max(1, dry.lengthM[i]));
        turb[i] = phi * turb[i] + dry.sigmaMs[i] * Math.sqrt(1 - phi * phi) * rand();
      }
    }
    const gust = oneMinusCosineGust(t, cfg.env.gustAmplitudeMs, cfg.gustStartS, cfg.gustDurationS);
    const along = meanW + turb[0] + gust;
    wind[0] = windTo[0] * along + windCross[0] * turb[1];
    wind[1] = turb[2];
    wind[2] = windTo[2] * along + windCross[2] * turb[1];

    // ================= state-derived quantities =================
    geom.update(fold);
    const k1 = deriv(x, out1); // also fills R, inp.vb for the current state
    const Rm = R.slice();
    const vb = [inp.vb[0], inp.vb[1], inp.vb[2]];
    const w = [x[10], x[11], x[12]];
    const phi = Math.atan2(-Rm[5], Rm[4]);
    const th = Math.asin(clamp(Rm[3], -1, 1));
    const psi = Math.atan2(Rm[6], Rm[0]);
    const pR = w[0], qR = w[2], rR = -w[1];
    const ias = Math.max(0, vb[0]);
    const vAir = Math.hypot(vb[0], vb[1], vb[2]);
    const out1BetaRaw = Math.atan2(vb[2], Math.max(1, Math.abs(vb[0])));
    const gs = Math.hypot(x[3], x[5]);
    const pn = x[0], pe = x[2], vn = x[3], ve = x[5], vy = x[4];
    // low-pass filtered aerodynamic force (world) and moment (body) used as feed-forward
    {
      const F = out1.Faero;
      const fw = [Rm[0] * F[0] + Rm[1] * F[1] + Rm[2] * F[2], Rm[3] * F[0] + Rm[4] * F[1] + Rm[5] * F[2], Rm[6] * F[0] + Rm[7] * F[1] + Rm[8] * F[2]];
      for (let i = 0; i < 3; i++) {
        faeroW[i] += (fw[i] - faeroW[i]) * (dt / 0.2);
        const m0 = out1.Maero[i] - (Bsurf[i][0] * surf[0] + Bsurf[i][1] * surf[1] + Bsurf[i][2] * surf[2]);
        maeroB[i] += (m0 - maeroB[i]) * (dt / 0.08);
      }
    }
    // surface effectiveness (numerical), refreshed at 10 Hz
    if (step % 10 === 0 && !onGround) {
      for (let j = 0; j < 3; j++) {
        const keep = surf[j];
        surf[j] = keep + 0.05;
        evalForces(m, geom, inp, outP, prevWingCl);
        surf[j] = keep;
        for (let i = 0; i < 3; i++) Bsurf[i][j] = (outP.M[i] - out1.M[i]) / 0.05;
      }
    }
    // mean thrust axis (world)
    let tbx = 0, tby = 0, tbz = 0;
    for (let k = 0; k < nR; k++) { tbx += geom.taxis[3 * k]; tby += geom.taxis[3 * k + 1]; tbz += geom.taxis[3 * k + 2]; }
    const tn = Math.hypot(tbx, tby, tbz);
    tbx /= tn; tby /= tn; tbz /= tn;
    const twy = Rm[3] * tbx + Rm[4] * tby + Rm[5] * tbz;
    const twn = Rm[0] * tbx + Rm[1] * tby + Rm[2] * tbz, twe = Rm[6] * tbx + Rm[7] * tby + Rm[8] * tbz;
    const chd = Math.cos(psi), shd = Math.sin(psi);
    const twFwd = twn * chd + twe * shd;

    // ================= guidance / phase logic =================
    const A = route[leg], B = route[leg + 1];
    const abn = B[0] - A[0], abe = B[1] - A[1];
    const abL = Math.hypot(abn, abe) || 1;
    const un = abn / abL, ue = abe / abL;
    const apn = pn - A[0], ape = pe - A[1];
    xtrack = un * ape - ue * apn;
    const alongTrack = un * apn + ue * ape;
    wpDist = Math.hypot(B[0] - pn, B[1] - pe);
    const lastLeg = leg === route.length - 2;
    let toGo = wpDist;
    for (let i = leg + 1; i < route.length - 1; i++) toGo += Math.hypot(route[i + 1][0] - route[i][0], route[i + 1][1] - route[i][1]);
    const distHome = Math.hypot(pn, pe);
    const tPhase = t - phaseT0;
    const cruiseAlt = cfg.cruiseAltitudeM;

    let mode: 'vtol' | 'trans' | 'fw' | 'back' = 'vtol';
    let spdOverride = 0;
    let posTgt: [number, number] = hoverPos;
    let hTgt = cfg.hoverHeightM;
    let vMaxH = 6;
    let climbUp = 2.5, climbDn = 1.5;
    switch (phase) {
      case Ph.TAKEOFF:
        hTgt = cfg.hoverHeightM;
        if (alt > cfg.hoverHeightM - 0.5) setPhase(Ph.HOVER_OUT, t);
        break;
      case Ph.HOVER_OUT:
        if (t >= cfg.takeoffHoverS) {
          setPhase(Ph.TRANSITION, t);
          transStartT = t; transStartAlt = alt; transMinAlt = alt; openPrev = 0;
        }
        break;
      case Ph.TRANSITION:
        mode = 'trans';
        transMinAlt = Math.min(transMinAlt, alt);
        if (fold < 1.5 * DEG && ias >= vOpen && vy > -1.5) {
          setPhase(Ph.CLIMB, t);
          hRef = alt; iAlt = 0; iSpd = 0; fwEntryT = t;
          ev(t, 'info', `İleri geçiş tamamlandı: ${(t - transStartT).toFixed(1)} s, irtifa kaybı ${(transStartAlt - transMinAlt).toFixed(1)} m`);
          flags.transDur = true;
        } else if (tPhase > 60) {
          outcome = 'İleri geçiş 60 s içinde tamamlanamadı';
          doneAt = t;
        }
        break;
      case Ph.CLIMB:
      case Ph.CRUISE:
      case Ph.DESCENT: {
        mode = 'fw';
        hTgt = cruiseAlt;
        // descent planning on the final leg
        const dDesc = ((cruiseAlt - approachAlt) / fwSink) * cfg.cruiseSpeedMs + dBack + 150;
        if (leg >= 1 && toGo < dDesc) {
          hTgt = approachAlt;
          if (phase !== Ph.DESCENT) { setPhase(Ph.DESCENT, t); tDesc = t; }
        }
        if (lastLeg) {
          if (wpDist < dBack || alongTrack > abL) {
            setPhase(Ph.BACK, t);
            backStartT = t;
            backFold = false;
            spdOverride = vDecel;
          }
        } else if (wpDist < Math.max(90, 4 * gs) || alongTrack > abL) {
          leg++;
          ev(t, 'info', `Waypoint ${leg} geçildi`);
        }
        if (phase === Ph.CLIMB && Math.abs(cruiseAlt - alt) < 5) setPhase(Ph.CRUISE, t);
        if (phase === Ph.DESCENT && mode === 'fw') hTgt = approachAlt;
        break;
      }
      case Ph.BACK:
        hTgt = approachAlt;
        if (!backFold) {
          // stage 0 — slow down in fixed-wing configuration (idle thrust) before folding
          mode = 'fw';
          spdOverride = vDecel;
          if (ias <= vDecel + 0.6 || tPhase > 15) { backFold = true; headingCmd = psi; }
          break;
        }
        mode = 'back';
        if (gs < 7 && fold > maxFold - 3 * DEG) {
          setPhase(Ph.HOVER_IN, t);
          ev(t, 'info', `Geri geçiş tamamlandı: ${(t - backStartT).toFixed(1)} s`);
        } else if (tPhase > 90) {
          outcome = 'Geri geçiş 90 s içinde tamamlanamadı';
          doneAt = t;
        }
        break;
      case Ph.HOVER_IN: {
        posTgt = home;
        hTgt = distHome > 30 ? approachAlt : cfg.hoverHeightM;
        vMaxH = 5;
        const descentTime = cfg.hoverHeightM / 0.9 + 4;
        if (distHome < 2.5 && gs < 0.8 && Math.abs(alt - cfg.hoverHeightM) < 2 && tPhase >= Math.max(5, cfg.landingHoverS - descentTime)) setPhase(Ph.LAND, t);
        break;
      }
      case Ph.LAND:
        posTgt = home;
        hTgt = -2;
        climbDn = clamp(0.3 + 0.22 * alt, 0.45, 1.2);
        break;
    }
    // align with the first leg shortly before the transition; otherwise weathervane (below)
    const aligning = phase === Ph.HOVER_OUT && t > cfg.takeoffHoverS - 8;
    if (aligning) headingCmd = trBearing;

    // ================= controller =================
    let Tdes = 0;
    let phiC = 0, thC = 0, rC = 0;
    let backStageA = false;
    const des = [0, 0, 0]; // desired angular acceleration (aerospace p, q, r)
    const sinceTrans = t - transStartT;
    altCmd = hTgt;
    spdCmd = 0;
    const vertThrust = (hT: number, up: number, dn: number) => {
      const vyC = clamp(1.2 * (hT - alt), -dn, up);
      const e = vyC - vy;
      iVy = clamp(iVy + e * dt, -4, 4);
      const aY = clamp(2.0 * e + 0.8 * iVy, -3, 2.5);
      return (mass * (G + aY) - faeroW[1]) / Math.max(0.25, twy);
    };
    if (mode === 'vtol' || mode === 'back') {
      // --- horizontal position / velocity → tilt
      let aN = 0, aE = 0;
      if (mode === 'vtol') {
        const en = posTgt[0] - pn, ee = posTgt[1] - pe;
        const dd = Math.hypot(en, ee);
        const vc = Math.min(vMaxH, 0.6 * dd);
        const vcn = dd > 1e-6 ? (vc * en) / dd : 0, vce = dd > 1e-6 ? (vc * ee) / dd : 0;
        aN = 1.4 * (vcn - vn); aE = 1.4 * (vce - ve);
      } else {
        // back-transition: decelerate along the current ground track
        const vTgt = Math.max(0, gs - 2.5);
        const k = gs > 0.5 ? vTgt / gs : 0;
        aN = 1.2 * (vn * k - vn); aE = 1.2 * (ve * k - ve);
      }
      const am = Math.hypot(aN, aE);
      const aLim = 3.5;
      if (am > aLim) { aN *= aLim / am; aE *= aLim / am; }
      // subtract the horizontal aerodynamic force (wind load) the tilt has to balance
      const fN = mass * aN - faeroW[0], fE = mass * aE - faeroW[2];
      const fFwd = fN * chd + fE * shd, fRight = -fN * shd + fE * chd;
      const tiltMax = (mode === 'back' ? 22 : 28) * DEG;
      thC = clamp(-Math.atan2(fFwd, W), -tiltMax, tiltMax);
      phiC = clamp(Math.atan2(fRight, W), -tiltMax, tiltMax);
      if (mode === 'back') {
        foldCmd = maxFold;
        thC = clamp(thC, -4 * DEG, 18 * DEG);
                Tdes = twy > 0.2 ? vertThrust(hTgt, 2, 2.5) : 0;
        if (fold < maxFold - 6 * DEG && ias > 9) {
          // stage A — the wing is still folding at speed: wings level, damp sideslip;
          // the folding (stalling) panels provide most of the deceleration
          backStageA = true;
          phiC = 0;
          // nose-up attitude: turns the thrust axis upwards sooner and adds braking
          thC = clamp(4 * DEG + (fold / maxFold) * 14 * DEG, 4 * DEG, 12 * DEG);
        } else backStageA = false;
      } else {
        foldCmd = maxFold;
        Tdes = vertThrust(hTgt, climbUp, climbDn);
      }
      // weathervaning (ArduPilot Q_WVANE-like): yaw into the sustained roll demand so that the folded
      // panels, which act as large side areas inside the propeller slipstream, stay edge-on to the wind
      if (mode === 'vtol' && !onGround && !aligning && alt > 2 && Math.abs(phiC) > 2 * DEG) {
        headingCmd = wrapPi(headingCmd + clamp(0.8 * phiC, -0.3, 0.3) * dt);
      }
      rC = clamp(1.5 * wrapPi(headingCmd - psi), -0.6, 0.6);
      if (mode === 'back' && backStageA) { rC = 1.6 * out1.beta; headingCmd = psi; }
      if (onGround) { iVy = 0; }
    } else if (mode === 'trans') {
      // --- forward transition: opening fraction scheduled on airspeed
      const ramp = clamp(sinceTrans / 2.5, 0, 1);
      let open = clamp(Math.max(ramp * tr.initialOpenFraction, ias / vOpen), 0, 1);
      // transition guard: opening tilts the thrust axis forward, so do not open further while the
      // aircraft is sinking or below its transition altitude (the wing is not carrying the load yet)
      const sinking = ias < 0.85 * vOpen && (vy < -0.7 || alt < transStartAlt - 1.2);
      if (sinking && open > openPrev) open = openPrev;
      openPrev = open;
      foldCmd = maxFold * (1 - open);
      spdCmd = cfg.cruiseSpeedMs;
      const wFw = clamp((ias - 0.55 * vOpen) / (0.35 * vOpen), 0, 1);
      // pitch: hold a positive attitude so that the opening wing carries load, altitude trimmed by pitch near the end
      const thAlt = clamp(0.03 * (transStartAlt - alt) - 0.04 * vy, -5 * DEG, 6 * DEG);
      thC = (3 + 5 * wFw) * DEG + wFw * thAlt;
      if (ias > 0.6 * vOpen) thC = Math.min(thC, th - out1.alpha + alphaMax);
      // lateral: keep the track
      const vRight = -vn * shd + ve * chd;
      phiC = clamp(Math.atan2(-0.8 * vRight, G), -12 * DEG, 12 * DEG);
      // mid-fold the panels form a steep "V" inside the slipstream: sideslip produces a rolling moment
      // larger than the rotor roll authority, so yaw is used to keep the sideslip at zero
      rC = clamp((ias > 3 ? 1.0 * out1BetaRaw : 0) + 0.6 * wrapPi(trBearing - psi), -0.5, 0.5);
      const tY = twy > 0.18 ? vertThrust(transStartAlt, 2, 2) : 0;
      const aX = clamp(0.8 * (spdCmd - ias), -1, 2);
      const fFwdAero = faeroW[0] * chd + faeroW[2] * shd;
      const tX = twFwd > 0.3 ? (mass * aX - fFwdAero) / twFwd : 0;
      // early: thrust carries the weight (tilt gives the acceleration); late: thrust also has to push
      Tdes = Math.min(open > 0.75 ? Math.max(tY, tX) : tY, 1.35 * W);
    } else {
      // --- fixed wing
      foldCmd = 0;
      spdCmd = spdOverride > 0 ? spdOverride : cfg.cruiseSpeedMs;
      // altitude reference with rate limit
      const up = 2.5, dn = fwSink;
      hRef += clamp(hTgt - hRef, -dn * dt, up * dt);
      altCmd = hRef;
      const ff = Math.abs(hTgt - hRef) > 0.5 ? (hTgt > hRef ? up : -dn) : 0;
      const hdotC = clamp(0.35 * (hRef - alt) + ff, -(dn + 1), up + 1);
      const vS = Math.max(ias, 0.8 * vOpen);
      const gamC = Math.asin(clamp(hdotC / vS, -0.25, 0.25));
      iAlt = clamp(iAlt + (hdotC - vy) * dt, -15, 15);
      const aTrim = interp(trimV, trimA, ias);
      thC = clamp(aTrim + gamC + 0.035 * (hdotC - vy) + 0.012 * iAlt, -12 * DEG, 14 * DEG);
      // angle-of-attack protection: never command a pitch that puts the wing beyond α_max
      thC = Math.min(thC, th - out1.alpha + alphaMax);
      // airspeed by thrust; airspeed has priority over altitude when slow (energy protection)
      const eV = spdCmd - ias;
      iSpd = clamp(iSpd + eV * dt, -30, 30);
      Tdes = interp(trimV, trimT, spdCmd) + W * Math.sin(gamC) + mass * (1.2 * eV + 0.1 * iSpd);
      if (eV > 0.8 && spdOverride === 0) thC -= clamp(0.025 * (eV - 0.8), 0, 7 * DEG);
      if (out1.stallFraction > 0.3 && spdOverride === 0) {
        // stall recovery: unload the wing and apply full power
        thC = Math.min(thC, th - out1.alpha + 0.6 * alphaMax);
        Tdes = Math.max(Tdes, 0.6 * W);
      }
      // L1 lateral guidance
      const L1 = Math.max(35, 0.3183 * 0.75 * 17 * gs);
      const nu1 = Math.asin(clamp(xtrack / L1, -0.7071, 0.7071));
      const xv = un * ve - ue * vn, lv = un * vn + ue * ve;
      const nu2 = Math.atan2(xv, lv);
      const latAcc = (-2.25 * gs * gs * Math.sin(clamp(nu1 + nu2, -Math.PI / 2, Math.PI / 2))) / L1;
      // bank limit keeps a stall margin: n ≤ 0.8·(V/Vs)², never more than 35°
      const nAllow = Math.max(1.02, 0.8 * (ias / vStall) ** 2);
      // gentle manoeuvring right after the transition and close to the ground
      const lowEnergy = t - fwEntryT < 6 || alt < 40;
      const bankLim = clamp(Math.acos(1 / nAllow), 8 * DEG, (lowEnergy ? 15 : 35) * DEG);
      const phiRaw = clamp(Math.atan2(latAcc, G), -bankLim, bankLim);
      phiC = phiCmdPrev + clamp(phiRaw - phiCmdPrev, -20 * DEG * dt, 20 * DEG * dt);
      phiCmdPrev = phiC;
      // coordinated turn + sideslip washout
      const psiDot = (G * Math.tan(phi)) / Math.max(10, vAir);
      rC = psiDot * Math.cos(phi) * Math.cos(th) + 1.0 * out1.beta;
      if (ias < tr.assistSpeedMs) assistTime += dt;
    }
    if (mode !== 'fw') phiCmdPrev = phi;
    if (onGround) { phiC = 0; thC = 0; rC = 0; }
    rollCmd = phiC; pitchCmd = thC;

    // attitude → rate → angular acceleration
    const fwLike = mode === 'fw' || ((mode === 'trans' || mode === 'back') && ias > 0.7 * vOpen);
    const kAng = fwLike ? 4.0 : 3.5;
    const kRate = fwLike ? 8 : 6.5;
    let pC = kAng * (phiC - phi);
    let qC = kAng * (thC - th);
    if (mode === 'fw') qC += ((G * Math.tan(phi)) / Math.max(10, vAir)) * Math.sin(phi) * Math.cos(th);
    pC = clamp(pC, -1.5, 1.5); qC = clamp(qC, -1.0, 1.0);
    const eP = pC - pR, eQ = qC - qR, eR = rC - rR;
    if (!onGround) {
      iP = clamp(iP + eP * dt, -1, 1); iQ = clamp(iQ + eQ * dt, -1, 1); iR = clamp(iR + eR * dt, -1, 1);
    }
    des[0] = kRate * eP + 3 * iP;
    des[1] = kRate * eQ + 3 * iQ;
    // fixed wing: yaw damper + sideslip feedback only (no integrator — the rudder authority of the V-tail is small)
    des[2] = mode === 'fw' ? clamp(1.8 * eR, -1.5, 1.5) : (fwLike ? 4 : 3) * eR + 1.5 * iR;
    if (mode === 'fw') iR = 0;
    if (!fwLike) des[2] = clamp(des[2], -1.2, 1.2); // yaw authority (reaction torque) is small: never let it starve roll / pitch
    // desired body moments (Mx roll, My = −yaw, Mz pitch) minus the aero moment not produced by controls
    const I = geom.I;
    const Mdes = onGround ? [0, 0, 0] : [I[0] * des[0] - maeroB[0], -I[4] * des[2] - maeroB[1], I[8] * des[1] - maeroB[2]];

    // ---- allocation
    const active = mode === 'fw' ? activeMask(m, cfg.cruiseMotors) : m.rotors.map(() => true);
    const nAct = active.filter(Boolean).length;
    const tMaxEach = m.rotors.map((_, k) => (active[k] ? interp(tmV, tmT, Math.max(0, out1.axialSpeed[k])) * clamp(vBatt / vNom, 0.7, 1.1) : 0));
    const tMin = mode === 'fw' ? 0 : 0.02 * (W / nR);
    // battery protection: cap the collective thrust so that the estimated electrical power stays
    // near the pack peak rating (momentum-theory estimate; exceedances are reported as events)
    const pCap = 0.9 * bat.peakC * capAh * vBatt * 0.82;
    const viEst = Math.sqrt(Math.max(1, Tdes) / (2 * rho * m.rotors[0].area * Math.max(1, nAct)));
    const tPowerCap = pCap / (Math.max(0, out1.axialSpeed[0]) + viEst / 0.75);
    Tdes = clamp(Math.min(Tdes, tPowerCap), mode === 'fw' ? 0 : nAct * tMin, tMaxEach.reduce((a, b) => a + b, 0));
    const u0: number[] = [];
    for (let k = 0; k < nR; k++) u0.push(active[k] ? Tdes / nAct : 0);
    u0.push(0, 0, 0);
    const nU = nR + 3;
    const Bm: number[][] = [new Array<number>(nU).fill(0), new Array<number>(nU).fill(0), new Array<number>(nU).fill(0), new Array<number>(nU).fill(0)];
    for (let k = 0; k < nR; k++) {
      if (!active[k]) continue;
      const hx = geom.hub[3 * k], hy = geom.hub[3 * k + 1], hz = geom.hub[3 * k + 2];
      const ax = geom.taxis[3 * k], ay = geom.taxis[3 * k + 1], az = geom.taxis[3 * k + 2];
      const sp = m.rotors[k].spin * kQ;
      Bm[0][k] = 1;
      Bm[1][k] = hy * az - hz * ay - sp * ax;
      Bm[2][k] = hz * ax - hx * az - sp * ay;
      Bm[3][k] = hx * ay - hy * ax - sp * az;
    }
    for (let j = 0; j < 3; j++) for (let i = 0; i < 3; i++) Bm[i + 1][nR + j] = Bsurf[i][j];
    // weights: differential thrust is blended out with airspeed; surfaces limited by deflection
    const sT = (W / nR) * clamp(1 - (ias / (1.2 * vOpen)) ** 2, 0.03, 1) * 0.8;
    const sU: number[] = [];
    for (let k = 0; k < nR; k++) sU.push(active[k] ? sT * sT : 0);
    for (let j = 0; j < 3; j++) sU.push(surfMax[j] * surfMax[j]);
    const yv = [Tdes, Mdes[0], Mdes[1], Mdes[2]];
    const lo = u0.map((_, i) => (i < nR ? (active[i] ? tMin : 0) : -surfMax[i - nR]));
    const hi = u0.map((_, i) => (i < nR ? tMaxEach[i] : surfMax[i - nR]));
    const u = u0.slice();
    const fixed = u0.map((_, i) => i < nR && !active[i]);
    satNow = 0;
    for (let pass = 0; pass < 3; pass++) {
      // residual with the current u
      const res = yv.map((y, i) => y - Bm[i].reduce((s, b, j) => s + b * u[j], 0));
      const M4: number[][] = [];
      for (let i = 0; i < 4; i++) {
        M4.push([]);
        for (let j = 0; j < 4; j++) {
          let s = 0;
          for (let c = 0; c < nU; c++) if (!fixed[c]) s += Bm[i][c] * sU[c] * Bm[j][c];
          M4[i].push(s);
        }
      }
      // priority (as in ArduPilot's motor mixer): roll / pitch first, then collective thrust, yaw last.
      // Once an actuator saturates, the thrust and yaw rows are progressively relaxed.
      const relT = pass === 0 ? 1e-9 : pass === 1 ? 0.5 : 2;
      const relY = pass === 0 ? 1e-9 : pass === 1 ? 5 : 30;
      M4[0][0] *= 1 + relT; M4[2][2] *= 1 + relY;
      for (let i = 0; i < 4; i++) M4[i][i] += 1e-9;
      const lam = solve(M4, res);
      let clipped = false;
      for (let c = 0; c < nU; c++) {
        if (fixed[c]) continue;
        let du = 0;
        for (let i = 0; i < 4; i++) du += sU[c] * Bm[i][c] * lam[i];
        const v = u[c] + du;
        if (v > hi[c] + 1e-9) { u[c] = hi[c]; fixed[c] = true; clipped = true; if (c < nR && !onGround) satNow = 1; }
        else if (v < lo[c] - 1e-9) { u[c] = lo[c]; fixed[c] = true; clipped = true; if (c < nR && mode !== 'fw' && !onGround) satNow = 1; }
        else u[c] = v;
      }
      if (!clipped) break;
    }
    // ---- actuators
    for (let k = 0; k < nR; k++) {
      if (!active[k] || (onGround && phase !== Ph.TAKEOFF) || phase === Ph.DONE) { duty[k] = 0; continue; }
      const va = Math.max(0, out1.axialSpeed[k]);
      const om = omegaForThrust(m, u[k], va, rho, D);
      const dCmd = dutyForOmega(m, om, va, rho, D, vBatt);
      // ESC acceleration limit (full range in 0.4 s; spool-up on the ground in ≈ 1.5 s)
      const slew = (onGround ? 0.35 : 2.5) * dt;
      duty[k] += clamp(dCmd - duty[k], -2.5 * dt, slew);
    }
    for (let j = 0; j < 3; j++) surfCmd[j] = u[nR + j];

    // ================= logging =================
    const pElec = out1.battCurrent * vBatt + 60;
    const battI = pElec / Math.max(1, vBatt);
    const cellV = vBatt / bat.cellsS;
    const cRate = battI / capAh;
    const spf = [out1.F[0] / mass, out1.F[1] / mass, out1.F[2] / mass];
    const pl = panelLoads(m, geom, out1, spf);
    const hm = pl.hinge;
    const nz = spf[1] / G;
    if (step % outEvery === 0) {
      const c = ch;
      c.t.push(t); c.phase.push(phase); c.pn.push(pn); c.pe.push(pe); c.alt.push(alt); c.vn.push(vn); c.ve.push(ve); c.vd.push(-vy);
      c.roll.push(phi / DEG); c.pitch.push(th / DEG); c.yaw.push(((psi / DEG) + 360) % 360); c.p.push(pR / DEG); c.q.push(qR / DEG); c.r.push(rR / DEG);
      c.ias.push(vAir); c.gs.push(gs); c.aoa.push(vb[0] > 6 ? out1.alpha / DEG : 0); c.beta.push(vb[0] > 6 ? out1.beta / DEG : 0); c.fold.push(fold / DEG); c.foldCmd.push(foldCmd / DEG);
      c.altCmd.push(altCmd); c.spdCmd.push(spdCmd); c.rollCmd.push(rollCmd / DEG); c.pitchCmd.push(pitchCmd / DEG);
      c.ail.push(surf[0] / DEG); c.elev.push(surf[1] / DEG); c.rud.push(surf[2] / DEG);
      c.battV.push(vBatt); c.battI.push(battI); c.soc.push((cfg.initialSoc - usedAh / capAh) * 100); c.power.push(pElec); c.cellV.push(cellV); c.cellT.push(cellTemp); c.cRate.push(cRate);
      c.thrustTot.push(out1.thrust.reduce((a, b) => a + b, 0)); c.hingeL.push(hm[0]); c.hingeR.push(hm[1]); c.nz.push(nz);
      c.windN.push(wind[0]); c.windE.push(wind[2]); c.windU.push(wind[1]); c.xtrack.push(xtrack); c.wpDist.push(wpDist);
      c.stall.push(out1.stallFraction); c.sat.push(satNow); c.cl.push(out1.wingCl); c.energyWh.push(energyJ / 3600);
      // specific force (body x fwd, y up, z right); on the ground the gear reaction carries the weight
      c.fx.push(onGround ? 0 : spf[0]); c.fy.push(onGround ? G : spf[1]); c.fz.push(onGround ? 0 : spf[2]);
      c.bendL.push(pl.bend[0]); c.bendR.push(pl.bend[1]); c.rho.push(rho); c.usedAh.push(usedAh); c.wp.push(leg);
      for (let k = 0; k < nR; k++) {
        c[`rpm${k + 1}`].push((x[13 + k] * 60) / (2 * Math.PI)); c[`thr${k + 1}`].push(duty[k] * 100); c[`thrust${k + 1}`].push(out1.thrust[k]);
        c[`cur${k + 1}`].push(out1.current[k]); c[`tmot${k + 1}`].push(tMotor[k]); c[`tesc${k + 1}`].push(tEsc[k]);
      }
    }
    // ---- accumulators / events
    energyJ += pElec * dt; phE += pElec * dt; phPk = Math.max(phPk, pElec);
    if (!onGround) { dist += gs * dt; phD += gs * dt; }
    minCellV = Math.min(minCellV, cellV); maxI = Math.max(maxI, battI);
    if (cRate > bat.maxContinuousC) tAboveC += dt;
    maxHinge = Math.max(maxHinge, Math.abs(hm[0]), Math.abs(hm[1]));
    if (!onGround) { maxNz = Math.max(maxNz, Math.abs(nz)); maxBank = Math.max(maxBank, Math.abs(phi) / DEG); }
    if (phase === Ph.CRUISE) { maxAltErr = Math.max(maxAltErr, Math.abs(alt - cruiseAlt)); maxXt = Math.max(maxXt, Math.abs(xtrack)); cruiseE += pElec * dt; cruiseT += dt; }
    if (mode === 'vtol' && !onGround) { hoverTime += dt; hoverE += pElec * dt; }
    if (satNow) satTime += dt;
    if (mode === 'fw' && out1.stallFraction > 0.3) stallTime += dt;
    if (cellV < bat.lowCellV) once('lowV', t, 'warn', `BATT_LOW_VOLT: hücre ${cellV.toFixed(2)} V < ${bat.lowCellV} V`);
    if (cellV < bat.criticalCellV) once('critV', t, 'alarm', `BATT_CRT_VOLT: hücre ${cellV.toFixed(2)} V < ${bat.criticalCellV} V`);
    if (cRate > bat.maxContinuousC) once('cRate', t, 'warn', `Batarya akımı sürekli limiti aştı: ${cRate.toFixed(2)} C > ${bat.maxContinuousC} C`);
    if (cRate > bat.peakC) once('cPeak', t, 'alarm', `Batarya akımı tepe limitini aştı: ${cRate.toFixed(2)} C > ${bat.peakC} C`);
    if (Math.abs(hm[0]) > spec.hingeActuator.maxTorqueNm) once('hinge', t, 'warn', `Menteşe momenti eyleyici limitini aştı: ${Math.abs(hm[0]).toFixed(0)} N·m > ${spec.hingeActuator.maxTorqueNm} N·m`);
    if (satNow) once('sat', t, 'warn', 'Rotor itki doyması');
    if (mode === 'fw' && out1.stallFraction > 0.3) once('stall', t, 'warn', `Sabit kanat modunda kanat stall: α = ${(out1.alpha / DEG).toFixed(1)}°, V = ${vAir.toFixed(1)} m/s`);
    if (mode === 'fw' && ias < tr.assistSpeedMs) once('assist', t, 'warn', `Hava hızı Q_ASSIST_SPEED altına düştü (${ias.toFixed(1)} < ${tr.assistSpeedMs} m/s)`);
    if (cfg.initialSoc - usedAh / capAh < bat.reserveSoc) once('reserve', t, 'warn', `SoC rezerv sınırının (%${(bat.reserveSoc * 100).toFixed(0)}) altına indi`);
    for (let k = 0; k < nR; k++) {
      maxTm = Math.max(maxTm, tMotor[k]); maxTe = Math.max(maxTe, tEsc[k]);
      if (tMotor[k] > spec.motor.maxTempC) once('tmot', t, 'warn', `Motor sıcaklığı limiti aştı (${tMotor[k].toFixed(0)} °C)`);
      if (tEsc[k] > spec.esc.maxTempC) once('tesc', t, 'warn', `ESC sıcaklığı limiti aştı (${tEsc[k].toFixed(0)} °C)`);
    }
    if (step % 2000 === 0 && onProgress) onProgress(Math.min(0.99, t / estTotal), phase);

    // ================= termination =================
    if (doneAt >= 0) break;
    if (!onGround && alt <= 0 && vy < 0 && phase === Ph.TAKEOFF && alt > -0.05) {
      // settled back onto the gear during lift-off: stay on the ground until thrust exceeds weight again
      onGround = true;
      x[1] = 0; x[3] = 0; x[4] = 0; x[5] = 0; x[10] = 0; x[11] = 0; x[12] = 0;
      x[6] = Math.cos(psi / 2); x[7] = 0; x[8] = -Math.sin(psi / 2); x[9] = 0;
    } else if (!onGround && alt <= 0 && vy < 0) {
      touchdown = -vy;
      if (phase === Ph.LAND) {
        completed = true;
        outcome = `Görev tamamlandı — iniş hızı ${touchdown.toFixed(2)} m/s`;
        ev(t, 'info', outcome);
      } else {
        outcome = `Yere çarpma (${PHASES[phase]} fazında, ${touchdown.toFixed(1)} m/s)`;
        ev(t, 'alarm', outcome);
      }
      setPhase(Ph.DONE, t);
      break;
    }
    if (cellV < bat.cellCutoffV - 0.15 || cfg.initialSoc - usedAh / capAh <= 0.005) {
      outcome = 'Batarya tükendi';
      ev(t, 'alarm', outcome);
      setPhase(Ph.DONE, t);
      break;
    }
    if (Math.abs(phi) > 100 * DEG || Math.abs(th) > 80 * DEG) {
      outcome = `Kontrol kaybı (${PHASES[phase]} fazında)`;
      ev(t, 'alarm', outcome);
      setPhase(Ph.DONE, t);
      break;
    }

    // ================= integrate =================
    if (onGround) {
      // rotors spin up; release when the vertical force exceeds the weight
      for (let k = 0; k < nR; k++) x[13 + k] = Math.max(0, x[13 + k] + k1[13 + k] * dt);
      const fy = Rm[3] * out1.F[0] + Rm[4] * out1.F[1] + Rm[5] * out1.F[2];
      if (fy > W) onGround = false;
    } else {
      const xs = new Array<number>(NX);
      for (let i = 0; i < NX; i++) xs[i] = x[i] + 0.5 * dt * k1[i];
      const k2 = deriv(xs, outS);
      for (let i = 0; i < NX; i++) xs[i] = x[i] + 0.5 * dt * k2[i];
      const k3 = deriv(xs, outS);
      for (let i = 0; i < NX; i++) xs[i] = x[i] + dt * k3[i];
      const k4 = deriv(xs, outS);
      for (let i = 0; i < NX; i++) x[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
      const qn = Math.hypot(x[6], x[7], x[8], x[9]);
      for (let i = 6; i < 10; i++) x[i] /= qn;
      for (let k = 0; k < nR; k++) if (x[13 + k] < 0) x[13 + k] = 0;
    }
    prevWingCl = out1.wingCl;
    // actuators (first order + rate limits)
    fold += clamp(foldCmd - fold, -foldRate * dt, foldRate * dt);
    for (let j = 0; j < 3; j++) {
      const rate = clamp((surfCmd[j] - surf[j]) / 0.04, -2.5, 2.5);
      surf[j] = clamp(surf[j] + rate * dt, -surfMax[j], surfMax[j]);
    }
    // battery
    usedAh += (battI * dt) / 3600;
    const soc = cfg.initialSoc - usedAh / capAh;
    vRc += ((battI * 0.4 * R0 - vRc) / 25) * dt;
    vBatt = bat.cellsS * interp(bat.ocvSoc, bat.ocvV, clamp(soc, 0, 1)) - battI * R0 - vRc;
    cellTemp += ((battI * battI * 1.4 * R0) / (product.mass.batteryKg * 950) - (cellTemp - ambient) / 3000) * dt;
    // motor / ESC thermal nodes
    for (let k = 0; k < nR; k++) {
      const i = out1.current[k];
      const loss = i * i * spec.motor.resistanceOhm + spec.motor.noLoadCurrentA * (x[13 + k] / m.kvRad);
      const cool = 1 + 0.06 * (Math.abs(out1.axialSpeed[k]) + 2 * out1.vi[k]);
      tMotor[k] += ((loss - ((tMotor[k] - ambient) * cool) / spec.motor.thermalResistanceKPerW) / spec.motor.thermalMassJPerK) * dt;
      const lossE = i * i * spec.esc.resistanceOhm + 0.012 * i * (x[13 + k] / m.kvRad);
      tEsc[k] += ((lossE - ((tEsc[k] - ambient) * cool) / escRth) / escC) * dt;
    }
  }
  if (!outcome) {
    outcome = 'Azami simülasyon süresi doldu';
    setPhase(Ph.DONE, ch.t[ch.t.length - 1] ?? 0);
  } else if (phase !== Ph.DONE) setPhase(Ph.DONE, ch.t[ch.t.length - 1] ?? 0);
  void tDesc; void mulMV;
  const tEnd = ch.t[ch.t.length - 1] ?? 0;
  const tp = phases.find((p) => p.phase === Ph.TRANSITION), bp = phases.find((p) => p.phase === Ph.BACK);
  return {
    productId: product.id, config: cfg, ch, nRotors: nR, phases, events,
    summary: {
      completed, outcome, durationS: tEnd, distanceM: dist, energyWh: energyJ / 3600, finalSoc: cfg.initialSoc - usedAh / capAh, minCellV,
      maxBatteryCurrentA: maxI, maxCRate: maxI / capAh, timeAboveContinuousCS: tAboveC, maxMotorTempC: maxTm, maxEscTempC: maxTe, maxHingeMomentNm: maxHinge,
      maxLoadFactor: maxNz, maxBankDeg: maxBank, maxAltErrorM: maxAltErr, maxCrossTrackM: maxXt, transitionAltLossM: transStartAlt - transMinAlt,
      transitionDurationS: tp ? tp.tEnd - tp.tStart : 0, backTransitionDurationS: bp ? bp.tEnd - bp.tStart : 0, saturationTimeS: satTime, stallTimeS: stallTime,
      assistTimeS: assistTime, touchdownSinkMs: touchdown, hoverTimeS: hoverTime, cruiseMeanPowerW: cruiseT > 0 ? cruiseE / cruiseT : 0, hoverMeanPowerW: hoverTime > 0 ? hoverE / hoverTime : 0,
    },
    assumptions: [...MISSION_ASSUMPTIONS, ...MODEL_ASSUMPTIONS],
    cdaFront: m.cdaFront,
    wallTimeMs: Date.now() - wall0,
  };
}
