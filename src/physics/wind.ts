/**
 * Wind models: steady wind, logarithmic boundary-layer profile, Dryden turbulence
 * (MIL-HDBK-1797 low-altitude model, seeded/reproducible) and discrete 1-cosine gust.
 */
import type { Result, TurbulenceLevel } from './types';

/** Reference height at which the scenario wind speed is specified. */
export const WIND_REF_HEIGHT_M = 10;
/** Surface roughness length (open terrain / sea with waves). */
export const ROUGHNESS_M = 0.03;

const FT = 0.3048;
const KT = 0.514444;

/** Deterministic PRNG (mulberry32). */
export function mulberry32(seed: number) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Standard normal samples via Box-Muller on a seeded uniform generator. */
export function gaussian(seed: number) {
  const u = mulberry32(seed);
  let spare: number | null = null;
  return () => {
    if (spare !== null) {
      const s = spare;
      spare = null;
      return s;
    }
    let a = u();
    if (a < 1e-12) a = 1e-12;
    const b = u();
    const r = Math.sqrt(-2 * Math.log(a));
    spare = r * Math.sin(2 * Math.PI * b);
    return r * Math.cos(2 * Math.PI * b);
  };
}

/** Mean wind at height h with the log profile anchored at the 10 m reference speed. */
export function windAtHeight(refSpeedMs: number, heightM: number, logProfile = true): number {
  if (!logProfile) return refSpeedMs;
  const h = Math.max(heightM, 1);
  return (refSpeedMs * Math.log(h / ROUGHNESS_M)) / Math.log(WIND_REF_HEIGHT_M / ROUGHNESS_M);
}

/** Wind speed at 20 ft (W20) for the MIL-HDBK-1797 low-altitude model. */
export const W20_MS: Record<TurbulenceLevel, number> = {
  none: 0,
  light: 15 * KT,
  moderate: 30 * KT,
  severe: 45 * KT,
};

/** Medium/high-altitude σ (≈2000 ft+, used above 1000 ft with linear blending). */
const SIGMA_HIGH_MS: Record<TurbulenceLevel, number> = { none: 0, light: 1.5, moderate: 3.0, severe: 6.0 };

export interface DrydenParams {
  sigmaMs: [number, number, number];
  lengthM: [number, number, number];
}

export function drydenParams(level: TurbulenceLevel, altitudeM: number): Result<DrydenParams> {
  const hft = Math.min(Math.max(altitudeM / FT, 10), 1000);
  const sw = 0.1 * W20_MS[level];
  const suv = sw / Math.pow(0.177 + 0.000823 * hft, 0.4);
  const Luv = (hft / Math.pow(0.177 + 0.000823 * hft, 1.2)) * FT;
  const Lw = hft * FT;
  let sigma: [number, number, number] = [suv, suv, sw];
  let L: [number, number, number] = [Luv, Luv, Lw];
  const altFt = altitudeM / FT;
  if (altFt > 1000) {
    const k = Math.min(1, (altFt - 1000) / 1000);
    const sh = SIGMA_HIGH_MS[level];
    const Lh = 1750 * FT;
    sigma = sigma.map((s) => s + (sh - s) * k) as [number, number, number];
    L = L.map((l) => l + (Lh - l) * k) as [number, number, number];
  }
  return {
    value: { sigmaMs: sigma, lengthM: L },
    assumptions: [
      'Dryden düşük irtifa modeli (MIL-HDBK-1797): σw = 0.1·W20, σu = σv = σw/(0.177+0.000823h)^0.4, Lw = h, Lu = Lv = h/(0.177+0.000823h)^1.2 (h ft).',
      'W20 = 15 / 30 / 45 kt (hafif / orta / şiddetli).',
      '1000 ft üzerinde σ ve L, 2000 ft yüksek irtifa değerlerine (L = 1750 ft) doğrusal geçişle bağlanır.',
      'Üç bileşen birinci derece Gauss–Markov süreci olarak üretilir (Dryden u-spektrumu ile aynı; v ve w için 2. derece şekillendirme yaklaşıklandı).',
      'Donmuş türbülans: taşıma hızı = max(hava hızı, ortalama rüzgâr, 1 m/s).',
    ],
  };
}

/** Seeded Dryden turbulence generator (body/wind-axis components u, v, w in m/s). */
export class DrydenTurbulence {
  private state: [number, number, number] = [0, 0, 0];
  private readonly rand: () => number;
  constructor(private readonly params: DrydenParams, seed: number) {
    this.rand = gaussian(seed);
    // start from the stationary distribution so statistics are valid from t = 0
    this.state = [0, 1, 2].map((i) => params.sigmaMs[i] * this.rand()) as [number, number, number];
  }
  step(dt: number, convectionSpeedMs: number): [number, number, number] {
    const V = Math.max(1, convectionSpeedMs);
    for (let i = 0; i < 3; i++) {
      const L = Math.max(1, this.params.lengthM[i]);
      const phi = Math.exp((-V * dt) / L);
      this.state[i] = phi * this.state[i] + this.params.sigmaMs[i] * Math.sqrt(1 - phi * phi) * this.rand();
    }
    return [...this.state] as [number, number, number];
  }
}

/** Discrete 1-cosine gust: v(t) = A/2·(1 − cos(2π(t − t0)/T)) for t0 ≤ t ≤ t0 + T. */
export function oneMinusCosineGust(tS: number, amplitudeMs: number, startS: number, durationS: number): number {
  if (amplitudeMs === 0 || tS < startS || tS > startS + durationS) return 0;
  return (amplitudeMs / 2) * (1 - Math.cos((2 * Math.PI * (tS - startS)) / durationS));
}
