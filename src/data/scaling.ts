/**
 * Geometric / mass scaling laws used to derive the 5 m and 10 m products from the 7 m reference.
 * Lengths ∝ s, areas ∝ s², masses ∝ s^2.5 (structure + battery), motor power ∝ s^2.75
 * (keeps hover T/W roughly constant: P_hover ∝ W^1.5 / D ∝ s^3.75 / s).
 * Every derived value is flagged `estimate: true`.
 */
import type { Part, PayloadOption, Product } from './schema';

export const MASS_EXPONENT = 2.5;
export const POWER_EXPONENT = 2.75;
export const AREA_EXPONENT = 2;

const r3 = (x: number) => Math.round(x * 1000) / 1000;
const r1 = (x: number) => Math.round(x * 10) / 10;

export function scaleLength(x: number, s: number) {
  return r3(x * s);
}
export function scaleMass(x: number, s: number) {
  return r3(x * Math.pow(s, MASS_EXPONENT));
}
export function scalePower(x: number, s: number) {
  return Math.round(x * Math.pow(s, POWER_EXPONENT));
}
export function scaleArea(x: number, s: number) {
  return Math.round(x * Math.pow(s, AREA_EXPONENT) * 10000) / 10000;
}

/** Derive a scaled product. Non-dimensional coefficients (cd0, e, FM, η ...) are kept. */
export function scaleProduct(ref: Product, s: number, id: string, name: string, sizeClass: string): Product {
  const g = ref.geometry;
  const parts: Part[] = ref.parts.map((p) => ({
    ...p,
    massKg: scaleMass(p.massKg, s),
    positionM: [scaleLength(p.positionM[0], s), scaleLength(p.positionM[1], s), scaleLength(p.positionM[2], s)],
    sizeM: p.sizeM ? [scaleLength(p.sizeM[0], s), scaleLength(p.sizeM[1], s), scaleLength(p.sizeM[2], s)] : undefined,
    estimate: true,
  }));
  const payloadOptions: PayloadOption[] = ref.payloadOptions.map((o) => ({
    ...o,
    massKg: r1(o.massKg * Math.pow(s, MASS_EXPONENT)),
    dragAreaM2: scaleArea(o.dragAreaM2, s),
    estimate: true,
  }));
  return {
    ...ref,
    id,
    name,
    sizeClass,
    derivedFrom: ref.id,
    scaleFactor: r3(s),
    geometry: {
      ...g,
      spanM: scaleLength(g.spanM, s),
      chordRootM: scaleLength(g.chordRootM, s),
      chordTipM: scaleLength(g.chordTipM, s),
      fuselageLengthM: scaleLength(g.fuselageLengthM, s),
      fuselageDiameterM: scaleLength(g.fuselageDiameterM, s),
      landingGearHeightM: scaleLength(g.landingGearHeightM, s),
      hinge: {
        ...g.hinge,
        positionM: [scaleLength(g.hinge.positionM[0], s), scaleLength(g.hinge.positionM[1], s), scaleLength(g.hinge.positionM[2], s)],
        estimate: true,
      },
      rotors: g.rotors.map((r) => ({ ...r, diameterM: scaleLength(r.diameterM, s), hubOffsetM: scaleLength(r.hubOffsetM, s) })),
      estimate: true,
    },
    mass: {
      emptyKg: scaleMass(ref.mass.emptyKg, s),
      batteryKg: scaleMass(ref.mass.batteryKg, s),
      payloadMaxKg: r1(ref.mass.payloadMaxKg * Math.pow(s, MASS_EXPONENT)),
      estimate: true,
    },
    propulsion: { ...ref.propulsion, motorMaxPowerW: scalePower(ref.propulsion.motorMaxPowerW, s), estimate: true },
    energy: { ...ref.energy, batteryWh: Math.round(ref.energy.batteryWh * Math.pow(s, MASS_EXPONENT)), estimate: true },
    aero: { ...ref.aero, estimate: true },
    parts,
    payloadOptions,
  };
}
