/**
 * Time-domain simulation (RK4, dt = 0.01 s). Runs inside a Web Worker in the app.
 *  - Hover: planar 3-DOF (horizontal position along the wind axis, altitude, tilt) with cascaded
 *    PID position hold and per-rotor thrust allocation / saturation.
 *  - Transition / cruise: point-mass 2-DOF (x, z) with fold schedule; thrust axis and wing lift
 *    effectiveness come from the hinge kinematics.
 */
import { Vector3 } from 'three';
import type { MissionSpec, PayloadOption, Product } from '../data/schema';
import { thrustDirection } from '../kinematics/hinge';
import { aspectRatio, projectedArea, wingAreaM2, wingLiftEffectiveness, windToDirection } from './airframe';
import { atmosphere } from './atmosphere';
import { cruisePerformance } from './cruise';
import { hoverGeometry, MAX_HOVER_TILT_DEG } from './hover';
import { partMass, partPosition } from './massProps';
import { axialShaftPowerW, diskAreaM2, hoverShaftPowerW, leastNorm, maxStaticThrustN, maxThrustAtSpeedN } from './rotor';
import { G0, type Environment, type FlightMode } from './types';
import { DrydenTurbulence, drydenParams, oneMinusCosineGust, windAtHeight } from './wind';

export interface SimInput {
  product: Product;
  payload: PayloadOption | null;
  env: Environment;
  mission: Pick<MissionSpec, 'hoverHeightM' | 'transitionS' | 'controlReserveFraction'>;
  mode: FlightMode;
  cruiseMotors: 2 | 4;
  durationS: number;
  seed: number;
  gustStartS: number;
  gustDurationS: number;
  dtS?: number;
  outputHz?: number;
}

export interface SimSummary {
  maxPosErrorM: number;
  rmsPosErrorM: number;
  maxAltErrorM: number;
  maxTiltDeg: number;
  saturationEvents: number;
  saturationTimeS: number;
  energyWh: number;
  peakPowerW: number;
  meanPowerW: number;
  positionLost: boolean;
  crashed: boolean;
}

/** Replay trace. Positions / wind in scene frame (x fwd, y up, z right); attitude [roll, pitch, yaw] deg. */
export interface SimTrace {
  source: 'internal' | 'external';
  label: string;
  mode: FlightMode;
  t: number[];
  pos: [number, number, number][];
  att: [number, number, number][];
  thrustN: number[][];
  saturated: boolean[][];
  powerW: number[];
  windMs: [number, number, number][];
  foldDeg: number[];
  airspeedMs: number[];
  /** Reference (target) position used for error metrics */
  refPos: [number, number, number];
  summary: SimSummary;
  assumptions: string[];
}

export const POSITION_LOST_M = 5;

type Vec = number[];
function rk4(f: (y: Vec) => Vec, y: Vec, h: number): Vec {
  const k1 = f(y);
  const k2 = f(y.map((v, i) => v + (h / 2) * k1[i]));
  const k3 = f(y.map((v, i) => v + (h / 2) * k2[i]));
  const k4 = f(y.map((v, i) => v + h * k3[i]));
  return y.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

const clamp = (x: number, a: number, b: number) => Math.min(b, Math.max(a, x));
const DEG = Math.PI / 180;

export function simulate(input: SimInput): SimTrace {
  return input.mode === 'hover' ? simulateHover(input) : simulateForward(input);
}

// ---------------------------------------------------------------------------------------------
// Hover 3-DOF
// ---------------------------------------------------------------------------------------------
export function simulateHover(input: SimInput): SimTrace {
  const { product, payload, env } = input;
  const dt = input.dtS ?? 0.01;
  const outEvery = Math.max(1, Math.round(1 / ((input.outputHz ?? 50) * dt)));
  const rho = atmosphere(env.altitudeM, env.isaDeltaK).value.densityKgM3;
  const geo = hoverGeometry(product, payload);
  const m = geo.mass.totalKg;
  const W = m * G0;
  const d = windToDirection(env.windFromDeg);
  const up = new Vector3(0, 1, 0);
  const k = new Vector3().crossVectors(d, up).normalize(); // horizontal axis perpendicular to plane
  // inertia about k through CG (point masses)
  let I = 0;
  for (const part of product.parts) {
    const mi = partMass(part, payload);
    const r = partPosition(product, part, geo.fold).sub(geo.cg);
    I += mi * (r.dot(d) ** 2 + r.y ** 2);
  }
  I = Math.max(I, 0.05 * m);
  const s = geo.arms.map((a) => a.dot(d));
  const p = geo.arms.map((a) => a.dot(k));
  const areas = product.geometry.rotors.map((r) => diskAreaM2(r.diameterM));
  const fm = product.propulsion.figureOfMerit;
  const tMax = areas.map((A) => maxStaticThrustN(product.propulsion.motorMaxPowerW, rho, A, fm));
  const projH = projectedArea(product, geo.fold, d);
  const projV = projectedArea(product, geo.fold, up);
  const cdaH = product.aero.sideCd * projH.areaM2;
  const cdaV = product.aero.sideCd * projV.areaM2;
  const cpDy = projH.centroid.y - geo.cg.y;
  const cpDs = projV.centroid.clone().sub(geo.cg).dot(d);
  const h0 = input.mission.hoverHeightM;

  const dry = drydenParams(env.turbulence, env.altitudeM + h0);
  const turb = new DrydenTurbulence(dry.value, input.seed);
  // gains (pole placement on the linearised loops)
  const wAtt = 6, zAtt = 0.8;
  const wPos = 0.9, zPos = 0.9;
  const wAlt = 1.6, zAlt = 0.9;
  const kp = { pos: wPos * wPos, alt: wAlt * wAlt, att: wAtt * wAtt };
  const kd = { pos: 2 * zPos * wPos, alt: 2 * zAlt * wAlt, att: 2 * zAtt * wAtt };
  const ki = { pos: 0.25 * wPos * wPos * wPos, alt: 0.25 * wAlt * wAlt * wAlt };

  // state: h, z(alt error from h0), phi, hd, zd, omega
  let y: Vec = [0, 0, 0, 0, 0, 0];
  let iPos = 0, iAlt = 0;
  const n = Math.round(input.durationS / dt);
  const trace = emptyTrace(input, [0, h0, 0]);
  let satPrev = false;
  let energyJ = 0;
  let crashed = false;
  let sumSq = 0, nSq = 0;
  let maxPos = 0, maxAlt = 0, maxTilt = 0, satEvents = 0, satTime = 0, peakP = 0, sumP = 0;

  for (let step = 0; step <= n; step++) {
    const t = step * dt;
    const [h, z, phi, hd, zd, om] = y;
    const turbUvw = turb.step(dt, Math.max(env.windSpeedMs, 1));
    const meanW = windAtHeight(env.windSpeedMs, h0 + z, env.logProfile);
    const gust = oneMinusCosineGust(t, env.gustAmplitudeMs, input.gustStartS, input.gustDurationS);
    const windH = meanW + turbUvw[0] + gust;
    const windV = turbUvw[2];

    // --- controller (zero-order hold over the step)
    const ePos = -h, eAlt = -z;
    iPos = clamp(iPos + ePos * dt, -20, 20);
    iAlt = clamp(iAlt + eAlt * dt, -10, 10);
    const aH = kp.pos * ePos - kd.pos * hd + ki.pos * iPos;
    const phiCmd = clamp(Math.atan2(aH, G0), -MAX_HOVER_TILT_DEG * DEG, MAX_HOVER_TILT_DEG * DEG);
    const aV = kp.alt * eAlt - kd.alt * zd + ki.alt * iAlt;
    const tCmd = clamp((m * (G0 + aV)) / Math.max(0.3, Math.cos(phi)), 0, 2 * tMax.reduce((a, b) => a + b, 0));
    const mCmd = I * (kp.att * (phiCmd - phi) - kd.att * om);
    // allocation: ΣT = Tcmd, −Σ s·T = M, Σ p·T = 0
    const req = leastNorm([s.map(() => 1), s.map((x) => -x), p], [tCmd, mCmd, 0]);
    const sat = req.map((tr, i) => tr > tMax[i] + 1e-9 || tr < -1e-9);
    const T = req.map((tr, i) => clamp(tr, 0, tMax[i]));
    const Ttot = T.reduce((a, b) => a + b, 0);
    const Mthr = -T.reduce((acc, ti, i) => acc + ti * s[i], 0);

    const deriv = (st: Vec): Vec => {
      const [, , ph, hdd, zdd, omg] = st;
      const vrh = windH - hdd;
      const vrv = windV - zdd;
      const fh = 0.5 * rho * cdaH * Math.abs(vrh) * vrh;
      const fv = 0.5 * rho * cdaV * Math.abs(vrv) * vrv;
      const ax = (Ttot * Math.sin(ph) + fh) / m;
      const az = (Ttot * Math.cos(ph) - W + fv) / m;
      const al = (Mthr + fh * cpDy - fv * cpDs) / I;
      return [hdd, zdd, omg, ax, az, al];
    };

    // power & recording
    const pElec = T.reduce((acc, ti, i) => acc + hoverShaftPowerW(ti, rho, areas[i], fm), 0) / product.propulsion.motorEscEfficiency;
    const anySat = sat.some(Boolean);
    if (anySat && !satPrev) satEvents++;
    if (anySat) satTime += dt;
    satPrev = anySat;
    energyJ += pElec * dt;
    peakP = Math.max(peakP, pElec);
    sumP += pElec;
    const posErr = Math.hypot(h, z);
    maxPos = Math.max(maxPos, posErr);
    maxAlt = Math.max(maxAlt, Math.abs(z));
    maxTilt = Math.max(maxTilt, Math.abs(phi) / DEG);
    sumSq += posErr * posErr;
    nSq++;
    if (step % outEvery === 0) {
      const world = d.clone().multiplyScalar(h).add(new Vector3(0, h0 + z, 0));
      const phiDeg = phi / DEG;
      const windW = d.clone().multiplyScalar(windH).add(new Vector3(0, windV, 0)).addScaledVector(k, turbUvw[1]);
      trace.t.push(t);
      trace.pos.push([world.x, world.y, world.z]);
      trace.att.push([phiDeg * d.z, -phiDeg * d.x, 0]);
      trace.thrustN.push(T);
      trace.saturated.push(sat);
      trace.powerW.push(pElec);
      trace.windMs.push([windW.x, windW.y, windW.z]);
      trace.foldDeg.push(geo.fold);
      trace.airspeedMs.push(Math.hypot(windH - hd, windV - zd));
    }
    if (crashed) continue;
    y = rk4(deriv, y, dt);
    if (h0 + y[1] <= 0) {
      crashed = true;
      y = [y[0], -h0, y[2], 0, 0, 0];
    }
  }
  trace.summary = {
    maxPosErrorM: maxPos,
    rmsPosErrorM: Math.sqrt(sumSq / Math.max(1, nSq)),
    maxAltErrorM: maxAlt,
    maxTiltDeg: maxTilt,
    saturationEvents: satEvents,
    saturationTimeS: satTime,
    energyWh: energyJ / 3600,
    peakPowerW: peakP,
    meanPowerW: sumP / (n + 1),
    positionLost: crashed || maxPos > POSITION_LOST_M,
    crashed,
  };
  trace.assumptions = [
    'Düzlemsel 3-DOF hover: rüzgâr ekseni boyunca yatay konum, irtifa ve eğim; yalpa ve düzlem dışı hareket yok.',
    'Kaskad PID: konum → eğim komutu (±35° sınırlı), irtifa → toplam itki, eğim → moment (kutup yerleştirme).',
    'İtki dağıtımı en küçük-norm; her rotor [0, Tmax] aralığına kırpılır, kırpılma = doyma olayı.',
    'Motor/ESC dinamiği ve gecikme ihmal edildi (anlık itki).',
    'Gövde sürüklenmesi kuadratik, yatay ve dikey izdüşüm alanlarıyla; basınç merkezi momenti dahil.',
    'Dryden v (düzlem dışı) bileşeni kaydedilir ancak dinamiğe uygulanmaz.',
    'Gust, ortalama rüzgâr yönünde uygulanır.',
    `Pozisyon kaybı: konum sapması > ${POSITION_LOST_M} m veya yere çarpma.`,
    'Entegrasyon RK4, dt = 0.01 s; kontrolcü adım boyunca sabit (ZOH).',
    ...dry.assumptions,
  ];
  return trace;
}

// ---------------------------------------------------------------------------------------------
// Transition / cruise point-mass model
// ---------------------------------------------------------------------------------------------
export function simulateForward(input: SimInput): SimTrace {
  const { product, payload, env } = input;
  const dt = input.dtS ?? 0.01;
  const outEvery = Math.max(1, Math.round(1 / ((input.outputHz ?? 50) * dt)));
  const rho = atmosphere(env.altitudeM, env.isaDeltaK).value.densityKgM3;
  const geo = hoverGeometry(product, payload);
  const m = geo.mass.totalKg;
  const W = m * G0;
  const S = wingAreaM2(product);
  const AR = aspectRatio(product);
  const maxFold = product.geometry.hinge.maxFoldDeg;
  const cruise = cruisePerformance({ product, payload, altitudeM: env.altitudeM, isaDeltaK: env.isaDeltaK, motors: input.cruiseMotors });
  const vCruise = input.mode === 'cruise' ? cruise.value.bestRangeSpeedMs : Math.max(1.3 * cruise.value.stallSpeedMs, cruise.value.minPowerSpeedMs);
  const transitionStart = 2;
  const transitionS = Math.max(2, input.mission.transitionS);
  const foldAt = (t: number) =>
    input.mode === 'cruise' ? 0 : maxFold * clamp(1 - (t - transitionStart) / transitionS, 0, 1);
  const rotorsActive = (fold: number) =>
    product.geometry.rotors.map((r) => !(fold < 1e-6 && input.cruiseMotors === 2 && r.foldableInCruise));
  const areas = product.geometry.rotors.map((r) => diskAreaM2(r.diameterM));
  const { figureOfMerit: fm, propEfficiency: etaP, motorEscEfficiency: etaM } = product.propulsion;
  const windDir = windToDirection(env.windFromDeg);
  const h0 = Math.max(input.mission.hoverHeightM, 30);
  const dry = drydenParams(env.turbulence, env.altitudeM + h0);
  const turb = new DrydenTurbulence(dry.value, input.seed);
  const cdPayload = (payload?.dragAreaM2 ?? 0);
  const frontArea = (fold: number) => product.aero.sideCd * projectedArea(product, fold, new Vector3(-1, 0, 0)).areaM2;
  const frontCache = new Map<number, number>();
  const frontAt = (fold: number) => {
    const key = Math.round(fold);
    if (!frontCache.has(key)) frontCache.set(key, frontArea(key));
    return frontCache.get(key)!;
  };
  const effCache = new Map<number, number>();
  const effAt = (fold: number) => {
    const key = Math.round(fold);
    if (!effCache.has(key)) effCache.set(key, wingLiftEffectiveness(product, key));
    return effCache.get(key)!;
  };

  // initial state: x, z(alt error), vx, vz
  const v0 = input.mode === 'cruise' ? vCruise : 0;
  let y: Vec = [0, 0, v0, 0];
  let iAlt = 0;
  const n = Math.round(input.durationS / dt);
  const trace = emptyTrace(input, [0, h0, 0]);
  let satPrev = false, energyJ = 0, crashed = false;
  let maxAlt = 0, sumSq = 0, satEvents = 0, satTime = 0, peakP = 0, sumP = 0;

  for (let step = 0; step <= n; step++) {
    const t = step * dt;
    const fold = foldAt(t);
    const ff = fold / maxFold;
    const active = rotorsActive(fold);
    const nAct = active.filter(Boolean).length;
    const td = new Vector3(0, 0, 0);
    product.geometry.rotors.forEach((r) => td.add(thrustDirection(product.geometry.hinge, r.wing, fold)));
    td.normalize();
    const tx = td.x, tz = td.y;
    const eff = effAt(fold);
    const [, z, vx, vz] = y;
    const turbUvw = turb.step(dt, Math.max(Math.abs(vx), env.windSpeedMs, 1));
    const meanW = windAtHeight(env.windSpeedMs, h0 + z, env.logProfile);
    const gust = oneMinusCosineGust(t, env.gustAmplitudeMs, input.gustStartS, input.gustDurationS);
    const windX = meanW * windDir.x + turbUvw[0];
    const windZ = turbUvw[2] + gust;
    const vTarget = input.mode === 'cruise' ? vCruise : vCruise * (1 - ff);

    // controller
    const eAlt = -z;
    iAlt = clamp(iAlt + eAlt * dt, -10, 10);
    const aZ = 2.25 * eAlt - 2.7 * vz + 0.6 * iAlt;
    const vax = vx - windX, vaz = vz - windZ;
    const V = Math.hypot(vax, vaz);
    const speedErr = vTarget - (vx - meanW * windDir.x);
    const aX = clamp(0.6 * speedErr, -3, 3);
    const q = 0.5 * rho * V * V;
    const cdaPar = (1 - ff) * (S * product.aero.cd0 + cdPayload) + ff * frontAt(fold) + 0.02 * S;
    const LmaxCl = product.aero.clMax * eff;
    // force demand: T·t + L·ẑ + F_drag = m·(aX, aZ + g)
    const dragEst = q * cdaPar;
    const Fx = m * aX + (V > 0.1 ? (dragEst * vax) / V : 0);
    const Fz = m * (aZ + G0) + (V > 0.1 ? (dragEst * vaz) / V : 0);
    // 2×2 least squares for [T, L] (normal equations), then clamp L to the available lift
    const a11 = tx * tx + tz * tz, a12 = tz, a22 = 1;
    const b1 = tx * Fx + tz * Fz, b2 = Fz;
    const det = a11 * a22 - a12 * a12;
    let L = Math.abs(det) > 1e-6 ? (a11 * b2 - a12 * b1) / det : 0;
    const Lmax = q * S * LmaxCl;
    L = clamp(L, 0, Lmax);
    let Treq = (tx * Fx + tz * (Fz - L)) / Math.max(1e-6, tx * tx + tz * tz);
    if (tx < 0.15) Treq = (Fz - L) / Math.max(0.2, tz);
    const vAxial = Math.max(0, vax * tx + vaz * tz);
    const tMaxEach = product.geometry.rotors.map((_, i) => maxThrustAtSpeedN(product, i, vAxial, rho));
    const perRotorReq = Treq / Math.max(1, nAct);
    const T = product.geometry.rotors.map((_, i) => (active[i] ? clamp(perRotorReq, 0, tMaxEach[i]) : 0));
    const sat = product.geometry.rotors.map((_, i) => active[i] && perRotorReq > tMaxEach[i]);
    const Ttot = T.reduce((a, b) => a + b, 0);
    const CL = q > 1 ? L / (q * S) : 0;

    const deriv = (st: Vec): Vec => {
      const [, , vxx, vzz] = st;
      const ax_ = vxx - windX, az_ = vzz - windZ;
      const VV = Math.hypot(ax_, az_);
      const qq = 0.5 * rho * VV * VV;
      const lift = qq * S * CL;
      const cdi = (CL * CL) / (Math.PI * product.aero.oswaldE * AR);
      const dragA = cdaPar + S * cdi * eff;
      const dx = VV > 0.05 ? (-qq * dragA * ax_) / VV : 0;
      const dz = VV > 0.05 ? (-qq * dragA * az_) / VV : 0;
      return [vxx, vzz, (Ttot * tx + dx) / m, (Ttot * tz + lift + dz - W) / m];
    };

    const pElec = T.reduce((acc, ti, i) => acc + (active[i] ? axialShaftPowerW(ti, vAxial, rho, areas[i], fm, etaP) : 0), 0) / etaM;
    const anySat = sat.some(Boolean);
    if (anySat && !satPrev) satEvents++;
    if (anySat) satTime += dt;
    satPrev = anySat;
    energyJ += pElec * dt;
    peakP = Math.max(peakP, pElec);
    sumP += pElec;
    maxAlt = Math.max(maxAlt, Math.abs(z));
    sumSq += z * z;
    if (step % outEvery === 0) {
      trace.t.push(t);
      trace.pos.push([y[0], h0 + z, 0]);
      trace.att.push([0, 0, 0]);
      trace.thrustN.push(T);
      trace.saturated.push(sat);
      trace.powerW.push(pElec);
      trace.windMs.push([windX, windZ, meanW * windDir.z + turbUvw[1]]);
      trace.foldDeg.push(fold);
      trace.airspeedMs.push(V);
    }
    if (crashed) continue;
    y = rk4(deriv, y, dt);
    if (h0 + y[1] <= 0) {
      crashed = true;
      y = [y[0], -h0, 0, 0];
    }
  }
  trace.summary = {
    maxPosErrorM: maxAlt,
    rmsPosErrorM: Math.sqrt(sumSq / (n + 1)),
    maxAltErrorM: maxAlt,
    maxTiltDeg: 0,
    saturationEvents: satEvents,
    saturationTimeS: satTime,
    energyWh: energyJ / 3600,
    peakPowerW: peakP,
    meanPowerW: sumP / (n + 1),
    positionLost: crashed || maxAlt > 2 * POSITION_LOST_M,
    crashed,
  };
  trace.assumptions = [
    'İleri uçuş noktasal kütle 2-DOF (x, z); gövde yatay (yunuslama = 0), lateral hareket yok.',
    `Geçiş: t = ${transitionStart} s'de başlar, katlanma açısı ${transitionS} s içinde doğrusal olarak max → 0.`,
    'İtki ekseni menteşe kinematiğinden; kanat kaldırma etkinliği (n·ŷ)² ile ölçeklenir.',
    'Kaldırma katsayısı kontrolcü tarafından anlık seçilir (0 ≤ CL ≤ CLmax·etkinlik); kaldırma düşey kabul edildi.',
    'Parazit sürüklenme: cruise CD0·S ile katlı izdüşüm alanı arasında katlanma oranıyla interpolasyon.',
    'Rotor gücü eksenel momentum teorisi; verim FM → pervane verimi geçişi (0–10 m/s).',
    'İtki aktif rotorlara eşit paylaştırılır; cruise 2 motor modunda katlanır pervaneler kapalı.',
    'İleri uçuşta gust düşey (1-cosine) uygulanır.',
    `Pozisyon kaybı: irtifa sapması > ${2 * POSITION_LOST_M} m veya yere çarpma.`,
    ...dry.assumptions,
  ];
  return trace;
}

function emptyTrace(input: SimInput, refPos: [number, number, number]): SimTrace {
  return {
    source: 'internal',
    label: `${input.product.id} — dahili model`,
    mode: input.mode,
    t: [], pos: [], att: [], thrustN: [], saturated: [], powerW: [], windMs: [], foldDeg: [], airspeedMs: [],
    refPos,
    summary: {
      maxPosErrorM: 0, rmsPosErrorM: 0, maxAltErrorM: 0, maxTiltDeg: 0, saturationEvents: 0, saturationTimeS: 0,
      energyWh: 0, peakPowerW: 0, meanPowerW: 0, positionLost: false, crashed: false,
    },
    assumptions: [],
  };
}
