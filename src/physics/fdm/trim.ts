/**
 * Trim, performance and stability analysis on the 6-DOF model:
 *  – level-flight trim vs airspeed (α, elevator, thrust, electrical power, L/D) for 4 and 2 motors,
 *  – parasite drag calibration against a target maximum L/D,
 *  – hover trim (per-rotor thrust, power, hinge moment),
 *  – stability & control derivatives by finite differences, neutral point / static margin,
 *  – dynamic modes from the numerically linearised rigid-body equations.
 */
import type { PayloadOption, Product } from '../../data/schema';
import { atmosphere } from '../atmosphere';
import { solve } from '../rotor';
import { dutyForOmega, evalForces, newForceOutput, omegaForThrust, type ForceOutput } from './forces';
import { buildModel, DEG, G, GeomState, mulMV, type FdmModel } from './model';

export interface StaticPoint {
  out: ForceOutput;
  omega: number[];
  duty: number[];
  elecPowerW: number;
  shaftPowerW: number;
}

/** Quasi-static evaluation: rotor speeds are set to deliver the requested thrusts. */
export function staticEval(m: FdmModel, geom: GeomState, vb: number[], w: number[], surf: number[], thrust: number[], rho: number, vBatt: number): StaticPoint {
  const out = newForceOutput(m.rotors.length);
  const omega: number[] = [], duty: number[] = [];
  m.rotors.forEach((r, k) => {
    const hx = geom.hub[3 * k], hy = geom.hub[3 * k + 1], hz = geom.hub[3 * k + 2];
    const ux = vb[0] + (w[1] * hz - w[2] * hy), uy = vb[1] + (w[2] * hx - w[0] * hz), uz = vb[2] + (w[0] * hy - w[1] * hx);
    const va = ux * geom.taxis[3 * k] + uy * geom.taxis[3 * k + 1] + uz * geom.taxis[3 * k + 2];
    const om = thrust[k] > 0 ? omegaForThrust(m, thrust[k], va, rho, r.diameter) : 0;
    omega.push(om);
    duty.push(thrust[k] > 0 ? dutyForOmega(m, om, va, rho, r.diameter, vBatt) : 0);
  });
  const inp = { vb, w, omega, duty, surf, rho, vBatt };
  let cl = 0;
  for (let i = 0; i < 4; i++) {
    evalForces(m, geom, inp, out, cl);
    cl = out.wingCl;
  }
  let shaft = 0;
  out.qAero.forEach((q, k) => (shaft += q * omega[k]));
  return { out, omega, duty, elecPowerW: out.battCurrent * vBatt, shaftPowerW: shaft };
}

export function activeMask(m: FdmModel, motors: 2 | 4): boolean[] {
  return m.rotors.map((r) => motors === 4 || !r.foldable);
}

export interface TrimPoint {
  speedMs: number;
  converged: boolean;
  alphaDeg: number;
  elevatorDeg: number;
  thrustN: number;
  rpm: number;
  elecPowerW: number;
  shaftPowerW: number;
  liftToDrag: number;
  cl: number;
  propEfficiency: number;
  stallFraction: number;
  advanceRatio: number;
}

/** Level-flight trim (fold = 0): solve α, elevator and thrust for force / pitching-moment balance. */
export function trimLevel(m: FdmModel, geom: GeomState, V: number, rho: number, motors: 2 | 4, vBatt: number): TrimPoint & { sp: StaticPoint; x: number[] } {
  geom.update(0);
  const mask = activeMask(m, motors);
  const nAct = mask.filter(Boolean).length;
  const W = m.massKg * G;
  let x = [4 * DEG, -2 * DEG, W / 15];
  const res = (y: number[]) => {
    const [a, de, T] = y;
    const sp = staticEval(m, geom, [V * Math.cos(a), -V * Math.sin(a), 0], [0, 0, 0], [0, de, 0], mask.map((on) => (on ? Math.max(0, T) / nAct : 0)), rho, vBatt);
    const F = sp.out.F, M = sp.out.M;
    return { r: [F[0] - W * Math.sin(a), F[1] - W * Math.cos(a), M[2]], sp };
  };
  let converged = false;
  let last = res(x);
  for (let it = 0; it < 30; it++) {
    const r0 = last.r;
    if (Math.abs(r0[0]) < 0.02 && Math.abs(r0[1]) < 0.05 && Math.abs(r0[2]) < 0.01) {
      converged = true;
      break;
    }
    const h = [0.002, 0.002, 0.5];
    const Jm = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let j = 0; j < 3; j++) {
      const xp = [...x];
      xp[j] += h[j];
      const rp = res(xp).r;
      for (let i = 0; i < 3; i++) Jm[i][j] = (rp[i] - r0[i]) / h[j];
    }
    const dx = solve(Jm, r0.map((v) => -v));
    const lim = [3 * DEG, 5 * DEG, W / 5];
    for (let j = 0; j < 3; j++) x[j] += Math.max(-lim[j], Math.min(lim[j], dx[j]));
    x = [Math.max(-10 * DEG, Math.min(22 * DEG, x[0])), Math.max(-25 * DEG, Math.min(25 * DEG, x[1])), Math.max(0, x[2])];
    last = res(x);
  }
  const sp = last.sp;
  const q = 0.5 * rho * V * V;
  const n = sp.omega.find((o) => o > 0) ?? 0;
  const useful = Math.max(0, x[2]) * V;
  return {
    speedMs: V, converged: converged && sp.out.stallFraction < 0.5, alphaDeg: x[0] / DEG, elevatorDeg: x[1] / DEG, thrustN: x[2],
    rpm: (n * 60) / (2 * Math.PI), elecPowerW: sp.elecPowerW, shaftPowerW: sp.shaftPowerW, liftToDrag: W / Math.max(1e-6, x[2]),
    cl: W / (q * m.Sref), propEfficiency: sp.shaftPowerW > 1 ? useful / sp.shaftPowerW : 0, stallFraction: sp.out.stallFraction,
    advanceRatio: n > 1 ? V / ((n / (2 * Math.PI)) * m.rotors[0].diameter) : 0, sp, x,
  };
}

/** Calibrate the fuselage / nacelle parasite drag so that the maximum trimmed L/D matches the target. */
export function calibrateDrag(m: FdmModel, rho: number, vBatt: number): { cdaFront: number; maxLD: number; speedMs: number } {
  const target = m.spec.targetMaxLD;
  const geom = new GeomState(m);
  const best = () => {
    let b = { ld: 0, v: 0 };
    for (let v = 16; v <= 40; v += 1) {
      const t = trimLevel(m, geom, v, rho, 4, vBatt);
      if (t.converged && t.liftToDrag > b.ld) b = { ld: t.liftToDrag, v };
    }
    return b;
  };
  if (!target) {
    const b = best();
    return { cdaFront: m.cdaFront, maxLD: b.ld, speedMs: b.v };
  }
  let lo = 0, hi = 0.12 * m.Sref;
  for (let i = 0; i < 14; i++) {
    m.cdaFront = (lo + hi) / 2;
    if (best().ld > target) lo = m.cdaFront;
    else hi = m.cdaFront;
  }
  m.cdaFront = (lo + hi) / 2;
  const b = best();
  return { cdaFront: m.cdaFront, maxLD: b.ld, speedMs: b.v };
}

export interface HoverTrim {
  converged: boolean;
  thrustN: number[];
  rpm: number[];
  currentA: number[];
  elecPowerW: number;
  shaftPowerW: number;
  hingeMomentNm: number[];
  downloadN: number;
  maxThrustN: number;
  thrustMargin: number;
  cgX: number;
  thrustCentroidX: number;
}

/** Maximum static thrust of one rotor limited by motor current and battery voltage. */
export function maxStaticThrust(m: FdmModel, rho: number, vBatt: number): number {
  const D = m.rotors[0].diameter;
  let lo = 0, hi = 1500;
  for (let i = 0; i < 40; i++) {
    const om = (lo + hi) / 2;
    const pr = staticProp(m, om, rho, D);
    const iNeed = pr.q / m.kt + m.spec.motor.noLoadCurrentA;
    const vNeed = om / m.kvRad + iNeed * m.rTot;
    if (iNeed <= m.spec.motor.maxCurrentA && vNeed <= vBatt) lo = om;
    else hi = om;
  }
  return staticProp(m, lo, rho, D).T;
}
function staticProp(m: FdmModel, om: number, rho: number, D: number) {
  const n = om / (2 * Math.PI);
  const T = m.spec.prop.ct[0] * rho * n * n * D ** 4;
  const P = m.spec.prop.cp[0] * rho * n ** 3 * D ** 5;
  return { T, q: om > 0 ? P / om : 0 };
}

export function trimHover(m: FdmModel, geom: GeomState, rho: number, vBatt: number): HoverTrim {
  geom.update(m.maxFoldRad);
  const W = m.massKg * G;
  const n = m.rotors.length;
  let T = new Array<number>(n).fill(W / n);
  const ev = (t: number[]) => {
    const sp = staticEval(m, geom, [0, 0, 0], [0, 0, 0], [0, 0, 0], t.map((v) => Math.max(0.01, v)), rho, vBatt);
    return { r: [sp.out.F[1] - W, sp.out.M[0], sp.out.M[1], sp.out.M[2]], sp };
  };
  let last = ev(T);
  let converged = false;
  for (let it = 0; it < 25; it++) {
    const r0 = last.r;
    if (Math.abs(r0[0]) < 0.05 && Math.abs(r0[1]) < 0.02 && Math.abs(r0[2]) < 0.02 && Math.abs(r0[3]) < 0.02) {
      converged = true;
      break;
    }
    // 4 equations, n unknowns: Newton step in the least-norm sense (n = 4 → square system)
    const Jm: number[][] = [[], [], [], []];
    for (let j = 0; j < n; j++) {
      const tp = [...T];
      tp[j] += 1;
      const rp = ev(tp).r;
      for (let i = 0; i < 4; i++) Jm[i][j] = rp[i] - r0[i];
    }
    const JJt = Jm.map((ri) => Jm.map((rj) => ri.reduce((s, v, k) => s + v * rj[k], 0)));
    for (let i = 0; i < 4; i++) JJt[i][i] += 1e-9;
    const y = solve(JJt, r0.map((v) => -v));
    T = T.map((t, j) => t + Jm.reduce((s, row, i) => s + row[j] * y[i], 0));
    last = ev(T);
  }
  const sp = last.sp;
  const tMax = maxStaticThrust(m, rho, vBatt);
  const vert = T.reduce((s, t, k) => s + t * geom.taxis[3 * k + 1], 0);
  return {
    converged: converged && T.every((t) => t > 0 && t < tMax),
    thrustN: T, rpm: sp.omega.map((o) => (o * 60) / (2 * Math.PI)), currentA: sp.out.current.slice(), elecPowerW: sp.elecPowerW, shaftPowerW: sp.shaftPowerW,
    hingeMomentNm: hingeTotal(m, geom, sp.out, [sp.out.F[0] / m.massKg, sp.out.F[1] / m.massKg, sp.out.F[2] / m.massKg]),
    downloadN: vert - W, maxThrustN: tMax, thrustMargin: 1 - Math.max(...T) / tMax,
    cgX: geom.cg[0], thrustCentroidX: geom.cg[0] + T.reduce((s, t, k) => s + t * geom.hub[3 * k], 0) / T.reduce((s, t) => s + t, 0),
  };
}

/** Hinge moment incl. the weight / inertial load of the panel (specific force f = F/m, body axes). */
export function hingeTotal(m: FdmModel, geom: GeomState, out: ForceOutput, specificForce: number[]): number[] {
  return [0, 1].map((b) => {
    const r = [geom.panelCg[b][0] - geom.pivotRel[b][0], geom.panelCg[b][1] - geom.pivotRel[b][1], geom.panelCg[b][2] - geom.pivotRel[b][2]];
    const f = specificForce.map((v) => -m.panelMassKg * v);
    const ax = m.axis[b];
    const mg = (r[1] * f[2] - r[2] * f[1]) * ax[0] + (r[2] * f[0] - r[0] * f[2]) * ax[1] + (r[0] * f[1] - r[1] * f[0]) * ax[2];
    return out.hinge[b] + mg;
  });
}

export interface Derivatives {
  CLalpha: number; CDalpha: number; Cmalpha: number;
  CYbeta: number; Clbeta: number; Cnbeta: number;
  Clp: number; Cnp: number; Cmq: number; CLq: number; Clr: number; Cnr: number;
  CldeltaA: number; CndeltaA: number; CmdeltaE: number; CLdeltaE: number; CndeltaR: number; CldeltaR: number;
  staticMargin: number;
  neutralPointXM: number;
  cgXM: number;
}

export interface Mode {
  name: string;
  real: number;
  imag: number;
  freqHz: number;
  damping: number;
  timeConstS: number;
  stable: boolean;
}

function coeffs(m: FdmModel, sp: StaticPoint, V: number, rho: number, alpha: number) {
  const q = 0.5 * rho * V * V, S = m.Sref;
  const F = sp.out.Faero, M = sp.out.Maero;
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return {
    CL: (F[0] * sa + F[1] * ca) / (q * S),
    CD: -(F[0] * ca - F[1] * sa) / (q * S),
    CY: F[2] / (q * S),
    Cl: M[0] / (q * S * m.bref),
    Cm: M[2] / (q * S * m.cref),
    Cn: -M[1] / (q * S * m.bref),
  };
}

export function derivatives(m: FdmModel, geom: GeomState, trim: ReturnType<typeof trimLevel>, rho: number, vBatt: number): Derivatives {
  const V = trim.speedMs;
  const a0 = trim.x[0], de0 = trim.x[1];
  const mask = activeMask(m, 4);
  const thrust = mask.map(() => trim.thrustN / mask.length);
  const at = (o: { a?: number; b?: number; p?: number; q?: number; r?: number; da?: number; de?: number; dr?: number }) => {
    const a = a0 + (o.a ?? 0), b = o.b ?? 0;
    const vb = [V * Math.cos(a) * Math.cos(b), -V * Math.sin(a) * Math.cos(b), V * Math.sin(b)];
    // aerospace rates → body: ω = (p, −r, q)
    const sp = staticEval(m, geom, vb, [o.p ?? 0, -(o.r ?? 0), o.q ?? 0], [o.da ?? 0, de0 + (o.de ?? 0), o.dr ?? 0], thrust, rho, vBatt);
    return coeffs(m, sp, V, rho, a);
  };
  const d = (key: 'a' | 'b' | 'p' | 'q' | 'r' | 'da' | 'de' | 'dr', h: number) => {
    const p = at({ [key]: h }), n = at({ [key]: -h });
    const out: Record<string, number> = {};
    for (const k of Object.keys(p) as (keyof typeof p)[]) out[k] = (p[k] - n[k]) / (2 * h);
    return out;
  };
  const dA = d('a', 0.5 * DEG), dB = d('b', 1 * DEG), dP = d('p', 0.05), dQ = d('q', 0.05), dR = d('r', 0.05);
  const dDa = d('da', 2 * DEG), dDe = d('de', 2 * DEG), dDr = d('dr', 2 * DEG);
  const kb = m.bref / (2 * V), kc = m.cref / (2 * V);
  const sm = -dA.Cm / dA.CL;
  return {
    CLalpha: dA.CL, CDalpha: dA.CD, Cmalpha: dA.Cm, CYbeta: dB.CY, Clbeta: dB.Cl, Cnbeta: dB.Cn,
    Clp: dP.Cl / kb, Cnp: dP.Cn / kb, Cmq: dQ.Cm / kc, CLq: dQ.CL / kc, Clr: dR.Cl / kb, Cnr: dR.Cn / kb,
    CldeltaA: dDa.Cl, CndeltaA: dDa.Cn, CmdeltaE: dDe.Cm, CLdeltaE: dDe.CL, CndeltaR: dDr.Cn, CldeltaR: dDr.Cl,
    staticMargin: sm, cgXM: geom.cg[0], neutralPointXM: geom.cg[0] - sm * m.cref,
  };
}

/** Rigid-body state derivative for x = [vx, vy, vz, wx, wy, wz, φ, θ] with frozen thrust / controls. */
function rigidDot(m: FdmModel, geom: GeomState, x: number[], surf: number[], thrust: number[], rho: number, vBatt: number): number[] {
  const vb = [x[0], x[1], x[2]], w = [x[3], x[4], x[5]];
  const phi = x[6], th = x[7];
  const sp = staticEval(m, geom, vb, w, surf, thrust, rho, vBatt);
  const F = sp.out.F, M = sp.out.M, h = sp.out.hRot;
  const gb = [-G * Math.sin(th), -G * Math.cos(phi) * Math.cos(th), G * Math.sin(phi) * Math.cos(th)];
  const cr = [w[1] * vb[2] - w[2] * vb[1], w[2] * vb[0] - w[0] * vb[2], w[0] * vb[1] - w[1] * vb[0]];
  const Iw = mulMV(geom.I, w).map((v, i) => v + h[i]);
  const wxIw = [w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0]];
  const wd = mulMV(geom.Iinv, [M[0] - wxIw[0], M[1] - wxIw[1], M[2] - wxIw[2]]);
  const p = w[0], q = w[2], r = -w[1];
  return [
    F[0] / m.massKg + gb[0] - cr[0], F[1] / m.massKg + gb[1] - cr[1], F[2] / m.massKg + gb[2] - cr[2], wd[0], wd[1], wd[2],
    p + Math.tan(th) * (q * Math.sin(phi) + r * Math.cos(phi)), q * Math.cos(phi) - r * Math.sin(phi),
  ];
}

/** Characteristic polynomial (Faddeev–LeVerrier) and roots (Durand–Kerner) of a small real matrix. */
export function eigenvalues(A: number[][]): { re: number; im: number }[] {
  const n = A.length;
  const mul = (X: number[][], Y: number[][]) => X.map((row) => Y[0].map((_, j) => row.reduce((s, v, k) => s + v * Y[k][j], 0)));
  const c = [1];
  let Mk = A.map((row) => row.map(() => 0));
  for (let k = 1; k <= n; k++) {
    Mk = mul(A, Mk).map((row, i) => row.map((v, j) => v + (i === j ? c[k - 1] : 0)));
    const AM = mul(A, Mk);
    let tr = 0;
    for (let i = 0; i < n; i++) tr += AM[i][i];
    c.push(-tr / k);
  }
  // roots of λ^n + c1 λ^(n-1) + ... + cn
  let z = Array.from({ length: n }, (_, i) => ({ re: Math.cos((2 * Math.PI * i) / n + 0.4) * (1 + i * 0.3), im: Math.sin((2 * Math.PI * i) / n + 0.4) * (1 + i * 0.3) }));
  const cm = (a: { re: number; im: number }, b: { re: number; im: number }) => ({ re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re });
  for (let it = 0; it < 400; it++) {
    let delta = 0;
    z = z.map((zi, i) => {
      let p = { re: 1, im: 0 };
      for (let k = 1; k <= n; k++) {
        p = cm(p, zi);
        p.re += c[k];
      }
      let den = { re: 1, im: 0 };
      z.forEach((zj, j) => {
        if (j !== i) den = cm(den, { re: zi.re - zj.re, im: zi.im - zj.im });
      });
      const d2 = den.re * den.re + den.im * den.im || 1e-30;
      const q = { re: (p.re * den.re + p.im * den.im) / d2, im: (p.im * den.re - p.re * den.im) / d2 };
      delta += Math.abs(q.re) + Math.abs(q.im);
      return { re: zi.re - q.re, im: zi.im - q.im };
    });
    if (delta < 1e-13) break;
  }
  return z;
}

function describe(name: string, e: { re: number; im: number }): Mode {
  const wn = Math.hypot(e.re, e.im);
  return {
    name, real: e.re, imag: Math.abs(e.im), freqHz: Math.abs(e.im) / (2 * Math.PI), damping: wn > 1e-9 ? -e.re / wn : 1,
    timeConstS: Math.abs(e.re) > 1e-9 ? 1 / Math.abs(e.re) : Infinity, stable: e.re < 0,
  };
}

export function dynamicModes(m: FdmModel, geom: GeomState, trim: ReturnType<typeof trimLevel>, rho: number, vBatt: number): Mode[] {
  const V = trim.speedMs, a = trim.x[0];
  const mask = activeMask(m, 4);
  const thrust = mask.map(() => trim.thrustN / mask.length);
  const surf = [0, trim.x[1], 0];
  const x0 = [V * Math.cos(a), -V * Math.sin(a), 0, 0, 0, 0, 0, a];
  const h = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.005, 0.005];
  const J: number[][] = Array.from({ length: 8 }, () => new Array<number>(8).fill(0));
  for (let j = 0; j < 8; j++) {
    const xp = [...x0], xn = [...x0];
    xp[j] += h[j];
    xn[j] -= h[j];
    const fp = rigidDot(m, geom, xp, surf, thrust, rho, vBatt), fn = rigidDot(m, geom, xn, surf, thrust, rho, vBatt);
    for (let i = 0; i < 8; i++) J[i][j] = (fp[i] - fn[i]) / (2 * h[j]);
  }
  const sub = (idx: number[]) => idx.map((i) => idx.map((j) => J[i][j]));
  const lon = eigenvalues(sub([0, 1, 5, 7]));
  const lat = eigenvalues(sub([2, 3, 4, 6]));
  const modes: Mode[] = [];
  const pairs = (ev: { re: number; im: number }[]) => {
    const osc = ev.filter((e) => e.im > 1e-6).sort((p, q) => Math.hypot(q.re, q.im) - Math.hypot(p.re, p.im));
    const real = ev.filter((e) => Math.abs(e.im) <= 1e-6).sort((p, q) => Math.abs(q.re) - Math.abs(p.re));
    return { osc, real };
  };
  const L = pairs(lon);
  if (L.osc[0]) modes.push(describe('Kısa periyot', L.osc[0]));
  if (L.osc[1]) modes.push(describe('Fugoid', L.osc[1]));
  L.real.forEach((e, i) => modes.push(describe(`Boylamsal gerçek kök ${i + 1}`, e)));
  const T = pairs(lat);
  if (T.osc[0]) modes.push(describe('Dutch roll', T.osc[0]));
  if (T.real[0]) modes.push(describe('Yalpa (roll) sönümü', T.real[0]));
  if (T.real[1]) modes.push(describe('Spiral', T.real[1]));
  T.real.slice(2).forEach((e, i) => modes.push(describe(`Yanal gerçek kök ${i + 3}`, e)));
  return modes;
}

export interface AnalysisReport {
  productId: string;
  massKg: number;
  rho: number;
  vlm: { clAlphaPerRad: number; spanEfficiency: number; aspectRatio: number; areaM2: number };
  calibration: { cdaFrontM2: number; maxLD: number; speedMs: number; target: number | null };
  stallSpeedMs: number;
  clMax: number;
  trim4: TrimPoint[];
  trim2: TrimPoint[];
  cruise4: TrimPoint;
  cruise2: TrimPoint | null;
  hover: HoverTrim;
  derivatives: Derivatives;
  modes: Mode[];
  inertia: number[];
  cgCruise: number[];
  cgHover: number[];
  assumptions: string[];
}

/** Prepare a calibrated model (cached parasite drag is returned for reuse by the mission simulation). */
export function prepareModel(product: Product, payload: PayloadOption | null, altitudeM: number, isaDeltaK: number, cdaFront?: number) {
  const m = buildModel(product, payload, { cdaFront });
  const rho = atmosphere(altitudeM, isaDeltaK).value.densityKgM3;
  const b = m.spec.battery;
  const vNom = b.cellsS * b.cellNominalV;
  const cal = cdaFront === undefined ? calibrateDrag(m, rho, vNom) : null;
  return { m, rho, vNom, cal };
}

export function analyze(product: Product, payload: PayloadOption | null, altitudeM: number, isaDeltaK: number, cruiseSpeedMs?: number): AnalysisReport {
  const { m, rho, vNom, cal } = prepareModel(product, payload, altitudeM, isaDeltaK);
  const geom = new GeomState(m);
  const vc = cruiseSpeedMs ?? m.spec.missionDefault.cruiseSpeedMs;
  const speeds: number[] = [];
  for (let v = 14; v <= 44; v += 1) speeds.push(v);
  const strip = (t: ReturnType<typeof trimLevel>): TrimPoint => {
    const { sp: _sp, x: _x, ...rest } = t;
    return rest;
  };
  const trim4 = speeds.map((v) => strip(trimLevel(m, geom, v, rho, 4, vNom)));
  const hasFoldable = m.rotors.some((r) => r.foldable);
  const trim2 = hasFoldable ? speeds.map((v) => strip(trimLevel(m, geom, v, rho, 2, vNom))) : [];
  // stall: lowest speed with a converged, mostly attached trim (0.1 m/s resolution)
  let vs = speeds[0];
  for (let v = 12; v <= 40; v += 0.1) {
    const t = trimLevel(m, geom, v, rho, 4, vNom);
    if (t.converged && t.stallFraction < 0.25) {
      vs = v;
      break;
    }
  }
  const W = m.massKg * G;
  const c4 = trimLevel(m, geom, vc, rho, 4, vNom);
  const c2 = hasFoldable ? trimLevel(m, geom, vc, rho, 2, vNom) : null;
  const der = derivatives(m, geom, c4, rho, vNom);
  const modes = dynamicModes(m, geom, c4, rho, vNom);
  geom.update(0);
  const cgCruise = [...geom.cg];
  const inertia = [geom.I[0], geom.I[4], geom.I[8]];
  const hover = trimHover(m, geom, rho, vNom);
  const cgHover = [...geom.cg];
  return {
    productId: product.id, massKg: m.massKg, rho,
    vlm: { clAlphaPerRad: m.vlm.clAlphaPerRad, spanEfficiency: m.vlm.spanEfficiency, aspectRatio: m.vlm.aspectRatio, areaM2: m.vlm.areaM2 },
    calibration: { cdaFrontM2: m.cdaFront, maxLD: cal?.maxLD ?? 0, speedMs: cal?.speedMs ?? 0, target: m.spec.targetMaxLD ?? null },
    stallSpeedMs: vs, clMax: W / (0.5 * rho * vs * vs * m.Sref), trim4, trim2, cruise4: strip(c4), cruise2: c2 ? strip(c2) : null, hover,
    derivatives: der, modes, inertia, cgCruise, cgHover,
    assumptions: MODEL_ASSUMPTIONS,
  };
}

export const MODEL_ASSUMPTIONS: string[] = [
  '6-DOF rijit gövde; katlanma açısına göre CG ve atalet tensörü tablodan (31 nokta) enterpole edilir. Panellerin katlanma hızından doğan atalet tepkileri ihmal edildi.',
  'Aerodinamik şerit teorisi: şerit başına 3B kaldırma eğimi, açık kanadın düzlemsel vortex-lattice çözümünden; kesit modeli doğrusal bölge + ±180° düz plaka normal kuvvet modeli (stall sonrası).',
  'İndüklenmiş sürüklenme şerit başına cl²/(π·e·AR); e veri dosyasındaki Oswald katsayısıdır. Gövde + nasel parazit sürüklenmesi, hedef (L/D)max değerini verecek şekilde kalibre edilir.',
  'Katlı konumda VLM yeniden çözülmez: şerit eğimleri açık kanat değerlerinde tutulur (geçiş için yaklaşım).',
  'Pervane: CT(J), CP(J) tabloları (eksenel ilerleme oranı); eğik akış kuvvetleri (H-kuvveti, P-faktörü) ihmal edildi. İndüklenmiş hız momentum teorisinden; akım tüpü içindeki şeritlere eklenir.',
  'Motor: DC eşdeğer devre (KV, iç direnç, boşta akım), akım sınırı; ESC görev oranı. Rejeneratif frenleme yok.',
  'Kuyruk, kanat iniş akımı ε = 2·CL/(π·AR) içinde çalışır. Gövde: küt cisim sürüklenmesi + Munk momentleri.',
  'Kararlılık türevleri trim etrafında merkezi sonlu farklarla; modlar boylamsal / yanal 4×4 alt sistemlerin özdeğerlerinden.',
  'Menteşe momenti = panel şeritlerinin aerodinamik momenti + rotor itki ve tepki torku + panel ağırlığı/atalet yükü (jiroskopik moment hariç).',
];
