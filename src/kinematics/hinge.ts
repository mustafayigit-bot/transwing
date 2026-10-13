/**
 * Transwing hinge kinematics (pure math, no React).
 *
 * Body frame (three.js friendly): x = forward (nose), y = up, z = right wing.
 * Each wing rotates about an inclined (dihedral) hinge axis through `hinge.positionM`.
 * Fold angle 0 = cruise, maxFoldDeg = full hover.
 *
 * With the default axis (az 57.5°, el -28.25°, 106.8°) the right wing span vector (+z)
 * is carried to a direction swept 65° aft and the propeller thrust axis (+x) to +y (up).
 * A single rotation therefore both folds the wing back and turns the props upward.
 */
import { Quaternion, Vector3 } from 'three';
import type { Hinge, Product, Rotor, Vec3 } from '../data/schema';

export type Side = 'L' | 'R';
const DEG = Math.PI / 180;

/** Unit hinge axis for the given wing side. The left axis is the mirror image (pseudo-vector) of the right one. */
export function hingeAxis(hinge: Hinge, side: Side): Vector3 {
  const az = hinge.axisAzimuthDeg * DEG;
  const el = hinge.axisElevationDeg * DEG;
  const ax = -Math.cos(el) * Math.cos(az);
  const ay = Math.sin(el);
  const azz = Math.cos(el) * Math.sin(az);
  // Mirror across the x-y plane: a_L = det(M)·M·a_R = (-ax, -ay, az)
  return side === 'R' ? new Vector3(ax, ay, azz).normalize() : new Vector3(-ax, -ay, azz).normalize();
}

export function hingePivot(hinge: Hinge, side: Side): Vector3 {
  const [x, y, z] = hinge.positionM;
  return new Vector3(x, y, side === 'R' ? Math.abs(z) : -Math.abs(z));
}

export function foldQuaternion(hinge: Hinge, side: Side, foldDeg: number): Quaternion {
  return new Quaternion().setFromAxisAngle(hingeAxis(hinge, side), foldDeg * DEG);
}

/** Transform a point given in the cruise body frame on the given wing to the folded position. */
export function foldPoint(hinge: Hinge, side: Side, foldDeg: number, p: Vec3 | Vector3): Vector3 {
  const v = Array.isArray(p) ? new Vector3(p[0], p[1], p[2]) : p.clone();
  const pivot = hingePivot(hinge, side);
  return v.sub(pivot).applyQuaternion(foldQuaternion(hinge, side, foldDeg)).add(pivot);
}

/** Thrust axis (unit) of a wing-mounted rotor at the given fold angle. Cruise thrust = +x. */
export function thrustDirection(hinge: Hinge, side: Side, foldDeg: number): Vector3 {
  return new Vector3(1, 0, 0).applyQuaternion(foldQuaternion(hinge, side, foldDeg));
}

/** Span direction (root -> tip) of a wing at the given fold angle. */
export function spanDirection(hinge: Hinge, side: Side, foldDeg: number): Vector3 {
  return new Vector3(0, 0, side === 'R' ? 1 : -1).applyQuaternion(foldQuaternion(hinge, side, foldDeg));
}

/** Exposed semi-span from hinge to tip. */
export function exposedSemiSpanM(product: Product): number {
  return product.geometry.spanM / 2 - Math.abs(product.geometry.hinge.positionM[2]);
}

/** Wing chord at a span fraction (0 = root at the hinge, 1 = tip), linear taper. */
export function chordAt(product: Product, spanFraction: number): number {
  const g = product.geometry;
  return g.chordRootM + (g.chordTipM - g.chordRootM) * spanFraction;
}

/** Rotor hub position in cruise frame (prop plane ahead of quarter-chord by hubOffsetM). */
export function rotorHubCruise(product: Product, rotor: Rotor): Vector3 {
  const pivot = hingePivot(product.geometry.hinge, rotor.wing);
  const r = rotor.spanFraction * exposedSemiSpanM(product);
  return new Vector3(pivot.x + rotor.hubOffsetM, pivot.y, pivot.z + (rotor.wing === 'R' ? r : -r));
}

export function rotorHub(product: Product, rotor: Rotor, foldDeg: number): Vector3 {
  return foldPoint(product.geometry.hinge, rotor.wing, foldDeg, rotorHubCruise(product, rotor));
}

/** Normalised fold 0..1 */
export function foldFraction(product: Product, foldDeg: number): number {
  return Math.min(1, Math.max(0, foldDeg / product.geometry.hinge.maxFoldDeg));
}

/**
 * Inverse design helper: hinge axis / angle that carries the span (+z) to a direction swept
 * `sweepDeg` aft of lateral (in the horizontal plane) and the thrust axis (+x) to +y.
 */
export function designHingeForSweep(sweepDeg: number): { axisAzimuthDeg: number; axisElevationDeg: number; maxFoldDeg: number } {
  const L = sweepDeg * DEG;
  const cosTheta = (Math.cos(L) - 1) / 2;
  const theta = Math.acos(cosTheta);
  const a = new Vector3(-Math.sin(L), -Math.sin(L), 1 + Math.cos(L)).normalize();
  return {
    axisAzimuthDeg: Math.atan2(a.z, -a.x) / DEG,
    axisElevationDeg: Math.asin(a.y) / DEG,
    maxFoldDeg: theta / DEG,
  };
}
