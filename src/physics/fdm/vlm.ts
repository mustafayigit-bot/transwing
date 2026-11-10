/**
 * Planar vortex-lattice method (one chordwise horseshoe per spanwise panel) for the unfolded wing.
 * Used to obtain the 3-D lift-curve slope, the spanwise loading and the inviscid span efficiency
 * directly from the planform geometry.
 *
 * Aero axes inside this file: x downstream, y to the right, z up.
 */
import { solve } from '../rotor';

export interface VlmInput {
  spanM: number;
  chordRootM: number;
  chordTipM: number;
  /** quarter-chord sweep, rad */
  sweepRad?: number;
  panels?: number;
}

export interface VlmResult {
  /** dCL/dα per rad (thin-airfoil 2π sections) */
  clAlphaPerRad: number;
  /** inviscid span efficiency e = CL² / (π·AR·CDi) */
  spanEfficiency: number;
  /** panel centre stations (y) and local lift-curve slope dcl/dα per rad */
  y: number[];
  clAlphaLocal: number[];
  areaM2: number;
  aspectRatio: number;
}

/** Velocity induced at P by a straight vortex segment A→B of unit circulation (Biot–Savart). */
function segment(px: number, py: number, pz: number, ax: number, ay: number, az: number, bx: number, by: number, bz: number, out: number[]) {
  const r1x = px - ax, r1y = py - ay, r1z = pz - az;
  const r2x = px - bx, r2y = py - by, r2z = pz - bz;
  const cx = r1y * r2z - r1z * r2y, cy = r1z * r2x - r1x * r2z, cz = r1x * r2y - r1y * r2x;
  const c2 = cx * cx + cy * cy + cz * cz;
  const r1 = Math.hypot(r1x, r1y, r1z), r2 = Math.hypot(r2x, r2y, r2z);
  if (c2 < 1e-12 || r1 < 1e-9 || r2 < 1e-9) return;
  const r0x = bx - ax, r0y = by - ay, r0z = bz - az;
  const k = ((r0x * r1x + r0y * r1y + r0z * r1z) / r1 - (r0x * r2x + r0y * r2y + r0z * r2z) / r2) / (4 * Math.PI * c2);
  out[0] += k * cx;
  out[1] += k * cy;
  out[2] += k * cz;
}

export function solveVlm(input: VlmInput): VlmResult {
  const N = input.panels ?? 40;
  const b = input.spanM;
  const sweep = input.sweepRad ?? 0;
  const chord = (y: number) => input.chordRootM + (input.chordTipM - input.chordRootM) * (Math.abs(y) / (b / 2));
  const xqc = (y: number) => Math.abs(y) * Math.tan(sweep);
  // cosine spacing concentrates panels at the tips
  const ys = Array.from({ length: N + 1 }, (_, i) => (-b / 2) * Math.cos((Math.PI * i) / N));
  const FAR = 200 * b;
  const A: number[][] = [];
  const ctrl: { x: number; y: number }[] = [];
  const mid: number[] = [];
  for (let i = 0; i < N; i++) {
    const ym = (ys[i] + ys[i + 1]) / 2;
    mid.push(ym);
    ctrl.push({ x: xqc(ym) + 0.5 * chord(ym), y: ym }); // 3/4-chord point (bound vortex at 1/4)
  }
  const horseshoe = (j: number, px: number, py: number, pz: number, boundToo: boolean) => {
    const v = [0, 0, 0];
    const ya = ys[j], yb = ys[j + 1];
    const xa = xqc(ya), xb = xqc(yb);
    segment(px, py, pz, xa + FAR, ya, 0, xa, ya, 0, v); // left trailing leg (from infinity)
    if (boundToo) segment(px, py, pz, xa, ya, 0, xb, yb, 0, v);
    segment(px, py, pz, xb, yb, 0, xb + FAR, yb, 0, v); // right trailing leg
    return v;
  };
  for (let i = 0; i < N; i++) {
    A.push(Array.from({ length: N }, (_, j) => horseshoe(j, ctrl[i].x, ctrl[i].y, 0, true)[2]));
  }
  // unit angle of attack, unit freestream: normal wash must cancel V·α → Σ w_ij Γ_j = −1
  const gamma = solve(A, Array.from({ length: N }, () => -1));
  let S = 0, L = 0, Di = 0;
  const clLocal: number[] = [];
  for (let i = 0; i < N; i++) {
    const dy = ys[i + 1] - ys[i];
    const c = chord(mid[i]);
    S += c * dy;
    L += gamma[i] * dy; // ρ V Γ dy with ρ = V = 1
    clLocal.push((2 * gamma[i]) / c);
    // downwash at the bound-vortex midpoint from the trailing legs only (near-field induced drag)
    let w = 0;
    for (let j = 0; j < N; j++) w += gamma[j] * horseshoe(j, xqc(mid[i]), mid[i], 0, false)[2];
    Di += -gamma[i] * w * dy;
  }
  const AR = (b * b) / S;
  const CL = (2 * L) / S;
  const CDi = (2 * Di) / S;
  return {
    clAlphaPerRad: CL,
    spanEfficiency: (CL * CL) / (Math.PI * AR * CDi),
    y: mid,
    clAlphaLocal: clLocal,
    areaM2: S,
    aspectRatio: AR,
  };
}

/** Local lift slope at station y by linear interpolation of the VLM result. */
export function vlmSlopeAt(v: VlmResult, y: number): number {
  const a = Math.abs(y);
  const n = v.y.length;
  if (a >= Math.abs(v.y[n - 1])) return v.clAlphaLocal[n - 1];
  for (let i = Math.floor(n / 2); i < n - 1; i++) {
    if (a >= v.y[i] && a <= v.y[i + 1]) {
      const t = (a - v.y[i]) / (v.y[i + 1] - v.y[i]);
      return v.clAlphaLocal[i] + t * (v.clAlphaLocal[i + 1] - v.clAlphaLocal[i]);
    }
  }
  return v.clAlphaLocal[Math.floor(n / 2)];
}
