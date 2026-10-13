/** Momentum-theory rotor helpers and least-norm thrust allocation. */
import type { Product } from '../data/schema';

export function diskAreaM2(diameterM: number) {
  return (Math.PI * diameterM * diameterM) / 4;
}

/** Ideal induced power in hover, P = T^1.5 / sqrt(2ρA). */
export function idealHoverPowerW(thrustN: number, rho: number, areaM2: number) {
  const t = Math.max(0, thrustN);
  return Math.pow(t, 1.5) / Math.sqrt(2 * rho * areaM2);
}

/** Shaft power of one rotor in hover including figure of merit. */
export function hoverShaftPowerW(thrustN: number, rho: number, areaM2: number, fm: number) {
  return idealHoverPowerW(thrustN, rho, areaM2) / fm;
}

/** Static thrust achievable with the given shaft power: T = (P·FM·sqrt(2ρA))^(2/3). */
export function maxStaticThrustN(shaftPowerW: number, rho: number, areaM2: number, fm: number) {
  return Math.pow(shaftPowerW * fm * Math.sqrt(2 * rho * areaM2), 2 / 3);
}

/**
 * Shaft power with an axial inflow component V (m/s) along the thrust axis.
 * Induced velocity from momentum theory in axial flight; efficiency blends from FM (static)
 * to propeller efficiency (≥ 10 m/s axial).
 */
export function axialShaftPowerW(thrustN: number, axialSpeedMs: number, rho: number, areaM2: number, fm: number, etaProp: number) {
  const t = Math.max(0, thrustN);
  const vh = Math.sqrt(t / (2 * rho * areaM2));
  const va = Math.max(0, axialSpeedMs);
  const vi = -va / 2 + Math.sqrt((va / 2) ** 2 + vh * vh);
  const blend = Math.min(1, va / 10);
  const eta = fm + (etaProp - fm) * blend;
  return (t * (va + vi)) / eta;
}

/** Maximum thrust of one rotor at axial speed V limited by the motor shaft power. */
export function maxThrustAtSpeedN(product: Product, rotorIndex: number, axialSpeedMs: number, rho: number) {
  const r = product.geometry.rotors[rotorIndex];
  const A = diskAreaM2(r.diameterM);
  const { motorMaxPowerW, figureOfMerit, propEfficiency } = product.propulsion;
  const tStatic = maxStaticThrustN(motorMaxPowerW, rho, A, figureOfMerit);
  if (axialSpeedMs < 1) return tStatic;
  // bisection on P(T) = Pmax
  let lo = 0, hi = tStatic;
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (axialShaftPowerW(mid, axialSpeedMs, rho, A, figureOfMerit, propEfficiency) > motorMaxPowerW) hi = mid;
    else lo = mid;
  }
  return lo;
}

/**
 * Least-norm solution of A·T = b (A is m×n, m ≤ 3) : T = Aᵀ (A Aᵀ)⁻¹ b.
 * Used to distribute a total thrust and moments over n rotors.
 */
export function leastNorm(A: number[][], b: number[]): number[] {
  const m = A.length;
  const n = A[0].length;
  const AAt: number[][] = Array.from({ length: m }, (_, i) =>
    Array.from({ length: m }, (_, j) => A[i].reduce((s, a, k) => s + a * A[j][k], 0)),
  );
  // Tikhonov regularisation keeps degenerate geometries (e.g. collinear rotors) solvable
  for (let i = 0; i < m; i++) AAt[i][i] += 1e-9 * (1 + AAt[i][i]);
  const y = solve(AAt, b);
  return Array.from({ length: n }, (_, k) => A.reduce((s, row, i) => s + row[k] * y[i], 0));
}

/** Gaussian elimination with partial pivoting. */
export function solve(M: number[][], b: number[]): number[] {
  const n = b.length;
  const a = M.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    const piv = a[c][c] || 1e-12;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = a[r][c] / piv;
      for (let k = c; k <= n; k++) a[r][k] -= f * a[c][k];
    }
  }
  return a.map((row, i) => row[n] / (a[i][i] || 1e-12));
}
