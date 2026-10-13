/** Mass, centre of gravity and inertia from the part list (point-mass model). */
import { Vector3 } from 'three';
import type { Part, PayloadOption, Product } from '../data/schema';
import { foldPoint, rotorHub } from '../kinematics/hinge';
import type { Result } from './types';

export interface MassProps {
  totalKg: number;
  cgM: [number, number, number];
  /** Diagonal inertia about CG (Ixx roll about x, Iyy yaw about y, Izz pitch about z), kg·m² */
  inertiaKgM2: [number, number, number];
  partMassKg: Record<string, number>;
}

export function payloadById(product: Product, payloadId: string | null | undefined): PayloadOption | null {
  if (!payloadId || payloadId === 'none') return null;
  return product.payloadOptions.find((p) => p.id === payloadId) ?? null;
}

/** Effective mass of a part for the selected payload (payload parts carry the option mass). */
export function partMass(part: Part, payload: PayloadOption | null): number {
  if (part.group === 'Payload') return payload ? payload.massKg : 0;
  return part.massKg;
}

/** Part CG position for a fold angle (wing-attached parts rotate about the hinge). */
export function partPosition(product: Product, part: Part, foldDeg: number): Vector3 {
  if (part.attach === 'body') return new Vector3(...part.positionM);
  return foldPoint(product.geometry.hinge, part.attach === 'wingL' ? 'L' : 'R', foldDeg, part.positionM);
}

export function massProperties(
  product: Product,
  payload: PayloadOption | null,
  foldDeg: number,
  hiddenParts: ReadonlySet<string> = new Set(),
): Result<MassProps> {
  let m = 0;
  const cg = new Vector3();
  const partMassKg: Record<string, number> = {};
  const positions: { m: number; p: Vector3 }[] = [];
  for (const part of product.parts) {
    if (hiddenParts.has(part.id)) continue;
    const mi = partMass(part, payload);
    partMassKg[part.id] = mi;
    const p = partPosition(product, part, foldDeg);
    positions.push({ m: mi, p });
    m += mi;
    cg.addScaledVector(p, mi);
  }
  if (m > 0) cg.divideScalar(m);
  let ixx = 0, iyy = 0, izz = 0;
  for (const { m: mi, p } of positions) {
    const d = p.clone().sub(cg);
    ixx += mi * (d.y * d.y + d.z * d.z);
    iyy += mi * (d.x * d.x + d.z * d.z);
    izz += mi * (d.x * d.x + d.y * d.y);
  }
  return {
    value: { totalKg: m, cgM: [cg.x, cg.y, cg.z], inertiaKgM2: [ixx, iyy, izz], partMassKg },
    assumptions: [
      'Parçalar nokta kütle olarak modellendi; kendi atalet momentleri ihmal edildi.',
      'Kanata bağlı parçalar (kanat, nasel, pervane) menteşe ekseni etrafında katlanma ile döner.',
      'Payload parçasının kütlesi seçili payload seçeneğinden alınır.',
    ],
  };
}

/** Rotor hub positions relative to CG at a fold angle. */
export function rotorArms(product: Product, foldDeg: number, cg: [number, number, number]): Vector3[] {
  const c = new Vector3(...cg);
  return product.geometry.rotors.map((r) => rotorHub(product, r, foldDeg).sub(c));
}
