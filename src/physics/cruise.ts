/** Cruise performance from a parabolic drag polar. */
import type { PayloadOption, Product } from '../data/schema';
import { aspectRatio, wingAreaM2 } from './airframe';
import { atmosphere } from './atmosphere';
import { massProperties } from './massProps';
import { G0, type Result } from './types';

/** Propeller efficiency penalty when all 4 props share the low cruise load. */
export const FOUR_MOTOR_ETA_FACTOR = 0.92;

export interface CruiseInput {
  product: Product;
  payload: PayloadOption | null;
  altitudeM: number;
  isaDeltaK: number;
  motors: 2 | 4;
}

export interface CruisePoint {
  speedMs: number;
  cl: number;
  cd: number;
  dragN: number;
  shaftPowerW: number;
  electricPowerW: number;
  /** electric energy per metre (J/m) */
  energyPerMetre: number;
}

export interface CruisePerformance {
  massKg: number;
  rhoKgM3: number;
  wingAreaM2: number;
  aspectRatio: number;
  stallSpeedMs: number;
  minPowerSpeedMs: number;
  bestRangeSpeedMs: number;
  maxSpeedMs: number;
  minPowerElectricW: number;
  bestRangeElectricW: number;
  availableShaftPowerW: number;
  bestLD: number;
  curve: CruisePoint[];
}

export function cruisePoint(input: CruiseInput, speedMs: number, massKg?: number, rho?: number): CruisePoint {
  const p = input.product;
  const m = massKg ?? massProperties(p, input.payload, 0).value.totalKg;
  const r = rho ?? atmosphere(input.altitudeM, input.isaDeltaK).value.densityKgM3;
  const S = wingAreaM2(p);
  const AR = aspectRatio(p);
  const q = 0.5 * r * speedMs * speedMs;
  const cl = (m * G0) / (q * S);
  const cdPayload = (input.payload?.dragAreaM2 ?? 0) / S;
  const cd = p.aero.cd0 + (cl * cl) / (Math.PI * p.aero.oswaldE * AR) + cdPayload;
  const D = q * S * cd;
  const etaProp = p.propulsion.propEfficiency * (input.motors === 4 ? FOUR_MOTOR_ETA_FACTOR : 1);
  const shaft = (D * speedMs) / etaProp;
  const elec = shaft / p.propulsion.motorEscEfficiency;
  return { speedMs, cl, cd, dragN: D, shaftPowerW: shaft, electricPowerW: elec, energyPerMetre: elec / speedMs };
}

export function cruisePerformance(input: CruiseInput): Result<CruisePerformance> {
  const p = input.product;
  const atm = atmosphere(input.altitudeM, input.isaDeltaK);
  const rho = atm.value.densityKgM3;
  const m = massProperties(p, input.payload, 0).value.totalKg;
  const S = wingAreaM2(p);
  const W = m * G0;
  const vs = Math.sqrt((2 * W) / (rho * S * p.aero.clMax));
  const avail = input.motors * p.propulsion.motorMaxPowerW;
  const curve: CruisePoint[] = [];
  const vMin = vs;
  const vMaxScan = Math.max(vs * 4, 80);
  let vMax = vMin;
  for (let v = vMin; v <= vMaxScan; v += 0.25) {
    const pt = cruisePoint(input, v, m, rho);
    curve.push(pt);
    if (pt.shaftPowerW <= avail) vMax = v;
  }
  // Operational speeds are kept ≥ 1.2·Vs (stall margin)
  const usable = curve.filter((c) => c.speedMs >= 1.2 * vs && c.shaftPowerW <= avail);
  const pool = usable.length ? usable : curve;
  const mp = pool.reduce((a, b) => (b.electricPowerW < a.electricPowerW ? b : a));
  const br = pool.reduce((a, b) => (b.energyPerMetre < a.energyPerMetre ? b : a));
  const ld = Math.max(...curve.map((c) => c.cl / c.cd));
  return {
    value: {
      massKg: m,
      rhoKgM3: rho,
      wingAreaM2: S,
      aspectRatio: aspectRatio(p),
      stallSpeedMs: vs,
      minPowerSpeedMs: mp.speedMs,
      bestRangeSpeedMs: br.speedMs,
      maxSpeedMs: vMax,
      minPowerElectricW: mp.electricPowerW,
      bestRangeElectricW: br.electricPowerW,
      availableShaftPowerW: avail,
      bestLD: ld,
      curve: curve.filter((c) => c.speedMs <= Math.min(vMaxScan, vMax * 1.25 + 5)),
    },
    assumptions: [
      ...atm.assumptions,
      'Parabolik sürüklenme poları: CD = CD0 + CL²/(π·e·AR) + payload CD·S / S.',
      'Referans alan brüt trapez kanat alanı; AR = b²/S.',
      'Stall hızı Vs = √(2W / (ρ S CLmax)); operasyonel hızlar ≥ 1.2·Vs.',
      `4 motorlu cruise'da pervane verimi ×${FOUR_MOTOR_ETA_FACTOR} (düşük yükte çalışma) kabul edildi; 2 motorda katlı pervanelerin sürüklenmesi CD0 içinde.`,
      'Mevcut güç = aktif motor sayısı × motorMaxPowerW (şaft).',
      'Sıkıştırılabilirlik ve Reynolds etkileri ihmal edildi.',
    ],
  };
}
