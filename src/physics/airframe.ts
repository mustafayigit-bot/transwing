/** Geometric aero quantities derived from the product data and the fold state. */
import { Vector3 } from 'three';
import type { Product } from '../data/schema';
import { exposedSemiSpanM, foldPoint, foldQuaternion, hingePivot } from '../kinematics/hinge';

/** Reference (gross trapezoidal) wing area, m². */
export function wingAreaM2(p: Product) {
  return (p.geometry.spanM * (p.geometry.chordRootM + p.geometry.chordTipM)) / 2;
}

export function aspectRatio(p: Product) {
  return (p.geometry.spanM * p.geometry.spanM) / wingAreaM2(p);
}

/** Exposed area of one wing panel (hinge to tip). */
export function wingPanelAreaM2(p: Product) {
  return (exposedSemiSpanM(p) * (p.geometry.chordRootM + p.geometry.chordTipM)) / 2;
}

/** Vertical tail / fin area assumed as 6 % of the wing reference area. */
export const FIN_AREA_FRACTION = 0.06;

interface AreaElement {
  areaM2: number;
  /** null = treat as a bluff body with given side/front areas */
  normal: Vector3 | null;
  centroid: Vector3;
  sideAreaM2?: number;
  frontAreaM2?: number;
  topAreaM2?: number;
}

function elements(p: Product, foldDeg: number): AreaElement[] {
  const g = p.geometry;
  const L = g.fuselageLengthM;
  const D = g.fuselageDiameterM;
  const els: AreaElement[] = [
    {
      areaM2: 0,
      normal: null,
      centroid: new Vector3(0, 0, 0),
      sideAreaM2: 0.85 * L * D,
      frontAreaM2: (Math.PI * D * D) / 4,
      topAreaM2: 0.85 * L * D,
    },
    { areaM2: FIN_AREA_FRACTION * wingAreaM2(p), normal: new Vector3(0, 0, 1), centroid: new Vector3(-L / 2 + 0.2 * L, D * 0.9, 0) },
  ];
  const semi = exposedSemiSpanM(p);
  for (const side of ['L', 'R'] as const) {
    const pivot = hingePivot(g.hinge, side);
    const q = foldQuaternion(g.hinge, side, foldDeg);
    const centroidCruise = new Vector3(pivot.x - 0.15 * g.chordRootM, pivot.y, pivot.z + (side === 'R' ? 1 : -1) * 0.45 * semi);
    els.push({
      areaM2: wingPanelAreaM2(p),
      normal: new Vector3(0, 1, 0).applyQuaternion(q),
      centroid: foldPoint(g.hinge, side, foldDeg, centroidCruise),
    });
  }
  return els;
}

/**
 * Projected (frontal) area seen by a flow along unit direction `dir` (body frame), and the
 * area-weighted centre of pressure. Wings are flat plates (|n·d|·A); fuselage is a bluff body.
 */
export function projectedArea(p: Product, foldDeg: number, dir: Vector3): { areaM2: number; centroid: Vector3 } {
  const d = dir.clone().normalize();
  let total = 0;
  const c = new Vector3();
  for (const e of elements(p, foldDeg)) {
    let a: number;
    if (e.normal) a = e.areaM2 * Math.abs(e.normal.dot(d));
    else a = (e.sideAreaM2 ?? 0) * Math.abs(d.z) + (e.frontAreaM2 ?? 0) * Math.abs(d.x) + (e.topAreaM2 ?? 0) * Math.abs(d.y);
    total += a;
    c.addScaledVector(e.centroid, a);
  }
  if (total > 0) c.divideScalar(total);
  return { areaM2: total, centroid: c };
}

/** Wing lift effectiveness vs fold: (n·ŷ)², 1 in cruise, ≈0 when the chord is vertical. */
export function wingLiftEffectiveness(p: Product, foldDeg: number): number {
  const n = new Vector3(0, 1, 0).applyQuaternion(foldQuaternion(p.geometry.hinge, 'R', foldDeg));
  return Math.max(0, n.y) ** 2;
}

/** Horizontal unit direction (body frame) towards which a wind "from" angle blows. 0° = from nose. */
export function windToDirection(windFromDeg: number): Vector3 {
  const b = (windFromDeg * Math.PI) / 180;
  return new Vector3(-Math.cos(b), 0, -Math.sin(b));
}
