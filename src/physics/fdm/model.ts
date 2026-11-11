/**
 * 6-DOF flight dynamics model set-up: converts product data into strips, rotors, mass tables.
 *
 * Body axes: x forward, y up, z right. World axes: X north, Y up, Z east.
 * Aerospace rates: p = ω_x (roll right), q = ω_z (pitch up), r = −ω_y (yaw right).
 *
 * Aerodynamics = strip theory (blade-element) with
 *   – per-strip 3-D lift slope from a planar vortex-lattice solution of the unfolded wing,
 *   – full ±180° section model (linear range blended into a flat-plate normal-force model),
 *   – propeller slipstream and wing downwash at the tail,
 *   – hinge kinematics: panel strips, rotors and masses rotate about the inclined hinge axis.
 */
import type { FdmSpec, Part, PayloadOption, Product } from '../../data/schema';
import { chordAtY, exposedSemiSpanM, hingeAxis, hingePivot, rotorHubCruise, wingQcX } from '../../kinematics/hinge';
import { aspectRatio, wingAreaM2 } from '../airframe';
import { partMass } from '../massProps';
import { solveVlm, vlmSlopeAt, type VlmResult } from './vlm';

export const DEG = Math.PI / 180;
export const G = 9.80665;

export interface Strip {
  /** 0 = fixed to fuselage, 1 = left folding panel, 2 = right folding panel */
  body: 0 | 1 | 2;
  /** 0 = wing, 1 = tail */
  kind: 0 | 1;
  p: number[];
  c: number[];
  n: number[];
  s: number[];
  area: number;
  chord: number;
  a: number;
  alpha0: number;
  aStallPos: number;
  aStallNeg: number;
  cd0: number;
  cm0: number;
  kInd: number;
  gAil: number;
  gElev: number;
  gRud: number;
}

export interface RotorDef {
  body: 1 | 2;
  hub: number[];
  spin: number;
  radius: number;
  area: number;
  diameter: number;
  foldable: boolean;
}

export interface MassSample {
  cg: number[];
  I: number[]; // 3×3 row-major about CG
  Iinv: number[];
  panelCg: number[][]; // [L, R] panel-group CG (body frame)
}

export interface FdmModel {
  product: Product;
  spec: FdmSpec;
  massKg: number;
  panelMassKg: number;
  maxFoldRad: number;
  axis: number[][]; // hinge axis [L, R]
  pivot: number[][];
  strips: Strip[];
  rotors: RotorDef[];
  massTable: MassSample[];
  Sref: number;
  bref: number;
  cref: number; // mean aerodynamic chord
  AR: number;
  /** x of wing quarter-chord (MAC) */
  xQc: number;
  /** parasite drag areas of fuselage + nacelles (m²): front, top, side */
  cdaFront: number;
  cdaTop: number;
  cdaSide: number;
  bodyPoint: number[];
  fuselageVolume: number;
  cd90: number;
  // propulsion
  kvRad: number; // rad/s per volt
  kt: number; // N·m per A
  rTot: number; // motor + ESC resistance
  vlm: VlmResult;
  tailSlope: number;
  flapTauAil: number;
  flapTauTail: number;
}

/** Flap effectiveness τ(cf/c) from thin-airfoil theory. */
export function flapEffectiveness(cf: number) {
  const th = Math.acos(2 * cf - 1);
  return 1 - (th - Math.sin(th)) / Math.PI;
}

function rodrigues(axis: number[], ang: number): number[] {
  const [x, y, z] = axis;
  const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c, t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}
export const rotationAbout = rodrigues;

export function mulMV(m: number[], v: number[]): number[] {
  return [m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]];
}

export function inv3(m: number[]): number[] {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  return [A / det, -(b * i - c * h) / det, (b * f - c * e) / det, B / det, (a * i - c * g) / det, -(a * f - c * d) / det, C / det, -(a * h - b * g) / det, (a * e - b * d) / det];
}

/** Intrinsic inertia (about the part's own CG) for large parts; point masses otherwise. */
function intrinsicInertia(part: Part, product: Product, m: number, spanDir: number[] | null): number[] {
  const g = product.geometry;
  const z = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  if (m <= 0) return z;
  if (part.kind === 'fuselage') {
    const L = g.fuselageLengthM, r = g.fuselageDiameterM / 2;
    const it = (m * L * L) / 12, ia = (m * r * r) / 2;
    return [ia, 0, 0, 0, it, 0, 0, 0, it];
  }
  if ((part.kind === 'wing' || part.kind === 'centerwing') && spanDir) {
    const L = part.kind === 'wing' ? exposedSemiSpanM(product) : 2 * Math.abs(g.hinge.positionM[2]);
    const k = (m * L * L) / 12;
    const [x, y, zz] = spanDir;
    return [k * (1 - x * x), -k * x * y, -k * x * zz, -k * x * y, k * (1 - y * y), -k * y * zz, -k * x * zz, -k * y * zz, k * (1 - zz * zz)];
  }
  if (part.sizeM) {
    const [a, b, c] = part.sizeM;
    return [(m * (b * b + c * c)) / 12, 0, 0, 0, (m * (a * a + c * c)) / 12, 0, 0, 0, (m * (a * a + b * b)) / 12];
  }
  return z;
}

export interface ModelOptions {
  /** override of the calibrated parasite drag area */
  cdaFront?: number;
}

export function buildModel(product: Product, payload: PayloadOption | null, opts: ModelOptions = {}): FdmModel {
  const spec = product.fdm;
  if (!spec) throw new Error(`${product.id}: 6-DOF modeli için "fdm" veri bloğu gerekli.`);
  const g = product.geometry;
  const hinge = g.hinge;
  const Sref = wingAreaM2(product);
  const AR = aspectRatio(product);
  const lam = g.chordTipM / g.chordRootM;
  const cref = (2 / 3) * g.chordRootM * ((1 + lam + lam * lam) / (1 + lam));
  const xQc = wingQcX(product);
  const sec = spec.section;
  const inc = g.incidenceDeg * DEG;
  const a0 = sec.liftSlopePerRad;
  const alpha0 = sec.alphaZeroLiftDeg * DEG;
  const vlm = solveVlm({ spanM: g.spanM, chordRootM: g.chordRootM, chordTipM: g.chordTipM });
  const kInd = 1 / (Math.PI * product.aero.oswaldE * AR);
  const strips: Strip[] = [];
  const hz = Math.abs(hinge.positionM[2]);
  const wy = hinge.positionM[1];
  const ail = g.aileron;
  const tauAil = ail ? flapEffectiveness(ail.chordFraction) : 0;

  const addWingStrip = (body: 0 | 1 | 2, side: 1 | -1, y1: number, y2: number) => {
    const ym = (y1 + y2) / 2;
    const chord = chordAtY(product, ym);
    const a = vlmSlopeAt(vlm, ym) * (a0 / (2 * Math.PI));
    // incidence: rotate chord / normal about the span axis (leading edge up)
    const c = [Math.cos(inc), Math.sin(inc), 0];
    const n = [-Math.sin(inc), Math.cos(inc), 0];
    let gAil = 0;
    if (ail && y2 > ail.startYM && y1 < ail.endYM) {
      const cover = (Math.min(y2, ail.endYM) - Math.max(y1, ail.startYM)) / (y2 - y1);
      gAil = -side * tauAil * cover; // +δa: right aileron trailing edge up → roll right
    }
    strips.push({
      body, kind: 0, p: [xQc, wy, side * ym], c, n, s: [0, 0, side], area: chord * (y2 - y1), chord, a, alpha0,
      aStallPos: alpha0 + sec.clMax / a, aStallNeg: alpha0 - (0.8 * sec.clMax) / a, cd0: sec.cd0, cm0: sec.cm0, kInd, gAil, gElev: 0, gRud: 0,
    });
  };
  const nCenter = hz > 0.2 ? 3 : 1;
  const nPanel = 8;
  for (const side of [-1, 1] as const) {
    for (let i = 0; i < nCenter; i++) addWingStrip(0, side, (hz * i) / nCenter, (hz * (i + 1)) / nCenter);
    const semi = g.spanM / 2;
    for (let i = 0; i < nPanel; i++) addWingStrip(side === 1 ? 2 : 1, side, hz + ((semi - hz) * i) / nPanel, hz + ((semi - hz) * (i + 1)) / nPanel);
  }
  // trapezoid vs reference area: scale strip areas so that Σ = Sref (carry-through / fairing area)
  const sumWing = strips.reduce((s, st) => s + st.area, 0);
  for (const st of strips) st.area *= Sref / sumWing;

  // tail
  let tailSlope = 0;
  let tauTail = 0;
  const tail = g.tail;
  if (tail) {
    const gam = tail.dihedralDeg * DEG;
    const panelArea = tail.areaM2 / 2;
    const arTail = ((2 * tail.panelSpanM) ** 2) / tail.areaM2;
    const aT0 = 2 * Math.PI * 0.95;
    tailSlope = aT0 / (Math.sqrt(1 + (aT0 / (Math.PI * arTail)) ** 2) + aT0 / (Math.PI * arTail));
    tauTail = flapEffectiveness(tail.controlChordFraction);
    const nT = 3;
    for (const side of [-1, 1] as const) {
      const s = [0, Math.sin(gam), side * Math.cos(gam)];
      const n = tail.type === 'v' ? [0, Math.cos(gam), -side * Math.sin(gam)] : [0, 1, 0];
      for (let i = 0; i < nT; i++) {
        const d = (tail.panelSpanM * (i + 0.5)) / nT;
        strips.push({
          body: 0, kind: 1, p: [tail.quarterChordXM, tail.rootYM + s[1] * d, s[2] * d + side * 0.02], c: [1, 0, 0], n, s,
          area: panelArea / nT, chord: panelArea / tail.panelSpanM, a: tailSlope, alpha0: 0,
          aStallPos: 1.1 / tailSlope, aStallNeg: -1.1 / tailSlope, cd0: 0.008, cm0: 0, kInd: 1 / (Math.PI * 0.9 * arTail),
          gAil: 0, gElev: -tauTail, gRud: tail.type === 'v' ? side * tauTail : 0,
        });
      }
    }
    if (tail.type === 'conventional') {
      // single fin for the conventional layout (30 % of tail area)
      strips.push({
        body: 0, kind: 1, p: [tail.quarterChordXM, tail.rootYM + 0.15, 0], c: [1, 0, 0], n: [0, 0, -1], s: [0, 1, 0], area: 0.3 * tail.areaM2,
        chord: 0.25, a: 2.5, alpha0: 0, aStallPos: 0.4, aStallNeg: -0.4, cd0: 0.008, cm0: 0, kInd: 0.1, gAil: 0, gElev: 0, gRud: tauTail,
      });
    }
  }

  // rotors: default spin pattern gives an X-quad torque layout in hover (inner pair vs outer pair)
  const rotors: RotorDef[] = g.rotors.map((r, i) => {
    const hub = rotorHubCruise(product, r);
    const inner = g.rotors.filter((o) => o.wing === r.wing).every((o) => o.spanFraction >= r.spanFraction);
    const def = (r.wing === 'L' ? 1 : -1) * (inner ? 1 : -1);
    void i;
    return {
      body: r.wing === 'L' ? 1 : 2, hub: [hub.x, hub.y, hub.z], spin: r.spin ?? def, radius: r.diameterM / 2,
      area: (Math.PI * r.diameterM * r.diameterM) / 4, diameter: r.diameterM, foldable: r.foldableInCruise,
    };
  });

  // mass tables vs fold
  const axisL = hingeAxis(hinge, 'L'), axisR = hingeAxis(hinge, 'R');
  const pivL = hingePivot(hinge, 'L'), pivR = hingePivot(hinge, 'R');
  const axis = [[axisL.x, axisL.y, axisL.z], [axisR.x, axisR.y, axisR.z]];
  const pivot = [[pivL.x, pivL.y, pivL.z], [pivR.x, pivR.y, pivR.z]];
  const maxFoldRad = hinge.maxFoldDeg * DEG;
  const NS = 31;
  const massTable: MassSample[] = [];
  let massKg = 0, panelMassKg = 0;
  for (const part of product.parts) {
    const m = partMass(part, payload);
    massKg += m;
    if (part.attach === 'wingR') panelMassKg += m;
  }
  for (let k = 0; k < NS; k++) {
    const fold = (maxFoldRad * k) / (NS - 1);
    const R = [rodrigues(axis[0], fold), rodrigues(axis[1], fold)];
    const items: { m: number; r: number[]; I0: number[] }[] = [];
    const pc = [[0, 0, 0], [0, 0, 0]];
    const pm = [0, 0];
    for (const part of product.parts) {
      const m = partMass(part, payload);
      if (m <= 0) continue;
      let r = [...part.positionM];
      let spanDir: number[] | null = part.kind === 'centerwing' ? [0, 0, 1] : null;
      if (part.attach !== 'body') {
        const b = part.attach === 'wingL' ? 0 : 1;
        const d = mulMV(R[b], [r[0] - pivot[b][0], r[1] - pivot[b][1], r[2] - pivot[b][2]]);
        r = [pivot[b][0] + d[0], pivot[b][1] + d[1], pivot[b][2] + d[2]];
        if (part.kind === 'wing') spanDir = mulMV(R[b], [0, 0, 1]);
        pm[b] += m;
        for (let q = 0; q < 3; q++) pc[b][q] += m * r[q];
      }
      items.push({ m, r, I0: intrinsicInertia(part, product, m, spanDir) });
    }
    const cg = [0, 0, 0];
    for (const it of items) for (let q = 0; q < 3; q++) cg[q] += (it.m * it.r[q]) / massKg;
    const I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (const it of items) {
      const dx = it.r[0] - cg[0], dy = it.r[1] - cg[1], dz = it.r[2] - cg[2];
      I[0] += it.m * (dy * dy + dz * dz); I[4] += it.m * (dx * dx + dz * dz); I[8] += it.m * (dx * dx + dy * dy);
      I[1] -= it.m * dx * dy; I[2] -= it.m * dx * dz; I[5] -= it.m * dy * dz;
      for (let q = 0; q < 9; q++) I[q] += it.I0[q];
    }
    I[3] = I[1]; I[6] = I[2]; I[7] = I[5];
    massTable.push({ cg, I, Iinv: inv3(I), panelCg: pc.map((v, b) => v.map((x) => x / Math.max(1e-9, pm[b]))) });
  }

  const L = g.fuselageLengthM, D = g.fuselageDiameterM;
  const kvRad = (spec.motor.kvRpmPerV * 2 * Math.PI) / 60;
  return {
    product, spec, massKg, panelMassKg, maxFoldRad, axis, pivot, strips, rotors, massTable, Sref, bref: g.spanM, cref, AR, xQc,
    cdaFront: opts.cdaFront ?? 0.012 * Sref,
    cdaTop: product.aero.sideCd * 0.85 * L * D,
    cdaSide: product.aero.sideCd * 0.85 * L * D,
    bodyPoint: [0.1 * L, 0, 0],
    fuselageVolume: 0.7 * Math.PI * (D / 2) ** 2 * L,
    cd90: sec.cd90,
    kvRad, kt: 1 / kvRad, rTot: spec.motor.resistanceOhm + spec.esc.resistanceOhm,
    vlm, tailSlope, flapTauAil: tauAil, flapTauTail: tauTail,
  };
}

/** Geometry at a given fold angle (strip frames, rotor hubs and axes relative to the CG). */
export class GeomState {
  fold = -1;
  cg = [0, 0, 0];
  I = new Array<number>(9).fill(0);
  Iinv = new Array<number>(9).fill(0);
  panelCg = [[0, 0, 0], [0, 0, 0]];
  /** chord axis (cruise +x) of each folding panel in body axes */
  panelChord = [[1, 0, 0], [1, 0, 0]];
  /** per strip: position rel. CG, chord axis, normal, span axis */
  sp: Float64Array;
  sc: Float64Array;
  sn: Float64Array;
  ss: Float64Array;
  hub: Float64Array; // per rotor rel. CG
  taxis: Float64Array;
  pivotRel = [[0, 0, 0], [0, 0, 0]];
  constructor(readonly m: FdmModel) {
    const n = m.strips.length;
    this.sp = new Float64Array(3 * n);
    this.sc = new Float64Array(3 * n);
    this.sn = new Float64Array(3 * n);
    this.ss = new Float64Array(3 * n);
    this.hub = new Float64Array(3 * m.rotors.length);
    this.taxis = new Float64Array(3 * m.rotors.length);
  }
  update(foldRad: number) {
    const m = this.m;
    const f = Math.min(m.maxFoldRad, Math.max(0, foldRad));
    if (Math.abs(f - this.fold) < 1e-7) return;
    this.fold = f;
    const t = (f / m.maxFoldRad) * (m.massTable.length - 1);
    const i0 = Math.min(m.massTable.length - 2, Math.floor(t));
    const u = t - i0;
    const A = m.massTable[i0], B = m.massTable[i0 + 1];
    for (let q = 0; q < 3; q++) {
      this.cg[q] = A.cg[q] + u * (B.cg[q] - A.cg[q]);
      for (let b = 0; b < 2; b++) this.panelCg[b][q] = A.panelCg[b][q] + u * (B.panelCg[b][q] - A.panelCg[b][q]) - this.cg[q];
    }
    for (let q = 0; q < 9; q++) this.I[q] = A.I[q] + u * (B.I[q] - A.I[q]);
    const inv = inv3(this.I);
    for (let q = 0; q < 9; q++) this.Iinv[q] = inv[q];
    const R = [rodrigues(m.axis[0], f), rodrigues(m.axis[1], f)];
    for (let b = 0; b < 2; b++) for (let q = 0; q < 3; q++) this.pivotRel[b][q] = m.pivot[b][q] - this.cg[q];
    for (let b = 0; b < 2; b++) this.panelChord[b] = mulMV(R[b], [1, 0, 0]);
    m.strips.forEach((st, k) => {
      let p = st.p, c = st.c, n = st.n, s = st.s;
      if (st.body !== 0) {
        const b = st.body - 1;
        const d = mulMV(R[b], [p[0] - m.pivot[b][0], p[1] - m.pivot[b][1], p[2] - m.pivot[b][2]]);
        p = [m.pivot[b][0] + d[0], m.pivot[b][1] + d[1], m.pivot[b][2] + d[2]];
        c = mulMV(R[b], c); n = mulMV(R[b], n); s = mulMV(R[b], s);
      }
      for (let q = 0; q < 3; q++) {
        this.sp[3 * k + q] = p[q] - this.cg[q];
        this.sc[3 * k + q] = c[q];
        this.sn[3 * k + q] = n[q];
        this.ss[3 * k + q] = s[q];
      }
    });
    m.rotors.forEach((r, k) => {
      const b = r.body - 1;
      const d = mulMV(R[b], [r.hub[0] - m.pivot[b][0], r.hub[1] - m.pivot[b][1], r.hub[2] - m.pivot[b][2]]);
      const ax = mulMV(R[b], [1, 0, 0]);
      for (let q = 0; q < 3; q++) {
        this.hub[3 * k + q] = m.pivot[b][q] + d[q] - this.cg[q];
        this.taxis[3 * k + q] = ax[q];
      }
    });
  }
}

/** Linear interpolation in a monotonic table (clamped). */
export function interp(xs: number[], ys: number[], x: number): number {
  const n = xs.length;
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xs[mid] <= x) lo = mid;
    else hi = mid;
  }
  return ys[lo] + ((ys[hi] - ys[lo]) * (x - xs[lo])) / (xs[hi] - xs[lo]);
}
