/** Hover performance (momentum theory) and hover wind limit (tilt / thrust / moment authority). */
import { Vector3 } from 'three';
import type { PayloadOption, Product } from '../data/schema';
import { rotorHub, thrustDirection } from '../kinematics/hinge';
import { projectedArea, windToDirection } from './airframe';
import { atmosphere } from './atmosphere';
import { massProperties } from './massProps';
import { diskAreaM2, hoverShaftPowerW, leastNorm, maxStaticThrustN } from './rotor';
import { G0, type Result } from './types';

export const MAX_HOVER_TILT_DEG = 35;

export interface HoverInput {
  product: Product;
  payload: PayloadOption | null;
  altitudeM: number;
  isaDeltaK: number;
  controlReserveFraction?: number;
}

export interface HoverPerformance {
  massKg: number;
  weightN: number;
  rhoKgM3: number;
  rotorThrustN: number[];
  rotorShaftPowerW: number[];
  maxRotorThrustN: number[];
  totalShaftPowerW: number;
  totalElectricPowerW: number;
  thrustToWeight: number;
  /** 1 − (most loaded rotor power / motor max power) */
  powerMargin: number;
  diskLoadingNM2: number;
  /** x offset of CG from rotor thrust centroid (+ = CG forward) */
  cgOffsetFromThrustCentroidM: number;
  trimFeasible: boolean;
}

/** Rotor arms (relative to CG) and thrust axes in full hover configuration. */
export function hoverGeometry(product: Product, payload: PayloadOption | null) {
  const fold = product.geometry.hinge.maxFoldDeg;
  const mp = massProperties(product, payload, fold);
  const cg = new Vector3(...mp.value.cgM);
  const arms = product.geometry.rotors.map((r) => rotorHub(product, r, fold).sub(cg));
  const axes = product.geometry.rotors.map((r) => thrustDirection(product.geometry.hinge, r.wing, fold));
  return { fold, mass: mp.value, cg, arms, axes, massAssumptions: mp.assumptions };
}

/**
 * Distribute a total thrust with required body moments (roll Mx, pitch Mz) over the rotors.
 * Body frame: x fwd, y up, z right. Thrust along +y at arm r gives M = r × F = (−z·T, 0, x·T).
 */
export function allocateHover(arms: Vector3[], totalN: number, mxNm: number, mzNm: number) {
  const A = [arms.map(() => 1), arms.map((a) => -a.z), arms.map((a) => a.x)];
  return leastNorm(A, [totalN, mxNm, mzNm]);
}

export function hoverPerformance(input: HoverInput): Result<HoverPerformance> {
  const { product } = input;
  const atm = atmosphere(input.altitudeM, input.isaDeltaK);
  const rho = atm.value.densityKgM3;
  const geo = hoverGeometry(product, input.payload);
  const W = geo.mass.totalKg * G0;
  const { figureOfMerit: fm, motorMaxPowerW, motorEscEfficiency } = product.propulsion;
  const areas = product.geometry.rotors.map((r) => diskAreaM2(r.diameterM));
  // vertical component of each thrust axis (≈1 in full hover)
  const cosTilt = geo.axes.map((a) => Math.max(0.05, a.y));
  const tVert = allocateHover(geo.arms, W, 0, 0);
  const thrust = tVert.map((t, i) => t / cosTilt[i]);
  const power = thrust.map((t, i) => hoverShaftPowerW(t, rho, areas[i], fm));
  const tMax = areas.map((A) => maxStaticThrustN(motorMaxPowerW, rho, A, fm));
  const totalShaft = power.reduce((s, x) => s + x, 0);
  const tw = tMax.reduce((s, t, i) => s + t * cosTilt[i], 0) / W;
  const worst = Math.max(...power.map((p) => p / motorMaxPowerW));
  const centroidX = geo.arms.reduce((s, a) => s + a.x, 0) / geo.arms.length;
  const totalArea = areas.reduce((s, a) => s + a, 0);
  return {
    value: {
      massKg: geo.mass.totalKg,
      weightN: W,
      rhoKgM3: rho,
      rotorThrustN: thrust,
      rotorShaftPowerW: power,
      maxRotorThrustN: tMax,
      totalShaftPowerW: totalShaft,
      totalElectricPowerW: totalShaft / motorEscEfficiency,
      thrustToWeight: tw,
      powerMargin: 1 - worst,
      diskLoadingNM2: W / totalArea,
      cgOffsetFromThrustCentroidM: -centroidX,
      trimFeasible: thrust.every((t, i) => t >= 0 && t <= tMax[i]),
    },
    assumptions: [
      ...atm.assumptions,
      'Rotor başına momentum teorisi: P = T^1.5 / √(2ρA), figure of merit ile düzeltildi.',
      'Hover tam katlı konfigürasyonda (maxFoldDeg) değerlendirilir.',
      'CG ofseti için diferansiyel itki en küçük-norm çözümüyle dağıtıldı; güç marjı en yüklü rotordan hesaplanır.',
      'Rotorlar arası etkileşim, yer etkisi ve gövde blokajı ihmal edildi.',
      'motorMaxPowerW sürekli şaft gücü kabul edildi; elektrik gücü = şaft / motor-ESC verimi.',
    ],
  };
}

export interface HoverWindCheck {
  windMs: number;
  dragN: number;
  tiltDeg: number;
  requiredThrustN: number;
  availableThrustN: number;
  rotorThrustN: number[];
  rotorUtilization: number[];
  limitedBy: 'ok' | 'tilt' | 'thrust' | 'moment';
  sideAreaM2: number;
}

/** Static force/moment balance holding position in a steady horizontal wind. */
export function hoverWindCheck(input: HoverInput, windMs: number, windFromDeg: number): HoverWindCheck {
  const { product } = input;
  const rho = atmosphere(input.altitudeM, input.isaDeltaK).value.densityKgM3;
  const geo = hoverGeometry(product, input.payload);
  const W = geo.mass.totalKg * G0;
  const reserve = input.controlReserveFraction ?? 0.15;
  const d = windToDirection(windFromDeg);
  const proj = projectedArea(product, geo.fold, d);
  const D = 0.5 * rho * windMs * windMs * product.aero.sideCd * proj.areaM2;
  const phi = Math.atan2(D, W);
  const Treq = Math.hypot(W, D);
  const areas = product.geometry.rotors.map((r) => diskAreaM2(r.diameterM));
  const tMax = areas.map((A) => maxStaticThrustN(product.propulsion.motorMaxPowerW, rho, A, product.propulsion.figureOfMerit));
  const usable = tMax.map((t) => t * (1 - reserve));
  // aerodynamic moment of the drag acting at the centre of pressure, about the CG
  const rcp = proj.centroid.clone().sub(geo.cg);
  const M = rcp.clone().cross(d.clone().multiplyScalar(D));
  const T = allocateHover(geo.arms, Treq, -M.x, -M.z);
  const util = T.map((t, i) => t / usable[i]);
  const availableThrustN = usable.reduce((s, t) => s + t, 0);
  let limitedBy: HoverWindCheck['limitedBy'] = 'ok';
  if ((phi * 180) / Math.PI > MAX_HOVER_TILT_DEG) limitedBy = 'tilt';
  else if (Treq > availableThrustN) limitedBy = 'thrust';
  else if (util.some((u) => u > 1 || u < 0)) limitedBy = 'moment';
  return {
    windMs,
    dragN: D,
    tiltDeg: (phi * 180) / Math.PI,
    requiredThrustN: Treq,
    availableThrustN,
    rotorThrustN: T,
    rotorUtilization: util,
    limitedBy,
    sideAreaM2: proj.areaM2,
  };
}

export interface HoverWindLimit {
  maxWindMs: number;
  limitedBy: HoverWindCheck['limitedBy'];
  worstDirectionDeg: number;
  worstDirectionMaxWindMs: number;
  atScenario: HoverWindCheck;
}

function bisectLimit(input: HoverInput, dirDeg: number) {
  if (hoverWindCheck(input, 0, dirDeg).limitedBy !== 'ok') return { v: 0, by: hoverWindCheck(input, 0, dirDeg).limitedBy };
  let lo = 0, hi = 80;
  if (hoverWindCheck(input, hi, dirDeg).limitedBy === 'ok') return { v: hi, by: 'ok' as const };
  for (let i = 0; i < 30; i++) {
    const mid = (lo + hi) / 2;
    if (hoverWindCheck(input, mid, dirDeg).limitedBy === 'ok') lo = mid;
    else hi = mid;
  }
  return { v: lo, by: hoverWindCheck(input, hi, dirDeg).limitedBy };
}

/** Maximum steady wind for position hold in the scenario direction, plus worst direction (45° steps). */
export function hoverWindLimit(input: HoverInput, windMs: number, windFromDeg: number): Result<HoverWindLimit> {
  const at = bisectLimit(input, windFromDeg);
  let worst = { dir: windFromDeg, v: at.v };
  for (let dir = 0; dir < 360; dir += 45) {
    const r = bisectLimit(input, dir);
    if (r.v < worst.v) worst = { dir, v: r.v };
  }
  return {
    value: {
      maxWindMs: at.v,
      limitedBy: at.by,
      worstDirectionDeg: worst.dir,
      worstDirectionMaxWindMs: worst.v,
      atScenario: hoverWindCheck(input, windMs, windFromDeg),
    },
    assumptions: [
      'Eğim açısı φ = atan(D_yan / W); gereken itki W / cos φ.',
      `Pozisyon tutma için maksimum eğim ${MAX_HOVER_TILT_DEG}° kabul edildi.`,
      `İtkinin %${Math.round((input.controlReserveFraction ?? 0.15) * 100)}'i kontrol rezervi olarak ayrıldı.`,
      'Yan izdüşüm alanı katlı geometriden (gövde + kanat panelleri |n·d|·A + dikey kuyruk) hesaplandı.',
      'Sürüklenme basınç merkezinde etki eder; oluşan moment diferansiyel itkiyle dengelenmelidir (moment otoritesi).',
      'Sabit (ortalama) rüzgâr; türbülans ve gust zaman alanı simülasyonunda değerlendirilir.',
      'Eğimin rotor kol geometrisine etkisi ihmal edildi (küçük açı).',
    ],
  };
}
