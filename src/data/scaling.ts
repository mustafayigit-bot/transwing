/**
 * Geometric / mass scaling laws used to derive the 5 m and 10 m products from the 7 m reference.
 * Lengths ∝ s, areas ∝ s², masses ∝ s^2.5 (structure + battery), motor power ∝ s^2.75
 * (keeps hover T/W roughly constant: P_hover ∝ W^1.5 / D ∝ s^3.75 / s).
 * Every derived value is flagged `estimate: true`.
 */
import type { FdmSpec, Part, PayloadOption, Product } from './schema';

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
      wingAreaM2: g.wingAreaM2 !== undefined ? scaleArea(g.wingAreaM2, s) : undefined,
      wingQcXM: g.wingQcXM !== undefined ? scaleLength(g.wingQcXM, s) : undefined,
      aileron: g.aileron ? { ...g.aileron, startYM: scaleLength(g.aileron.startYM, s), endYM: scaleLength(g.aileron.endYM, s) } : undefined,
      tail: g.tail
        ? { ...g.tail, areaM2: scaleArea(g.tail.areaM2, s), quarterChordXM: scaleLength(g.tail.quarterChordXM, s), rootYM: scaleLength(g.tail.rootYM, s), panelSpanM: scaleLength(g.tail.panelSpanM, s) }
        : undefined,
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
    fdm: ref.fdm ? scaleFdm(ref.fdm, s) : undefined,
  };
}

/**
 * Scaling of the 6-DOF data block at constant bus voltage:
 * thrust ∝ s^2.5, rotor speed ∝ s^-0.75, torque ∝ s^3.5, power and current ∝ s^2.75,
 * hence KV ∝ s^-0.75 and resistances ∝ s^-2.75; airspeeds ∝ √(wing loading) ∝ s^0.25.
 */
export function scaleFdm(f: FdmSpec, s: number): FdmSpec {
  const sig = (x: number) => Number(x.toPrecision(4));
  const pw = (e: number) => Math.pow(s, e);
  return {
    ...f,
    prop: { ...f.prop, pitchM: scaleLength(f.prop.pitchM, s), rotorInertiaKgM2: sig(f.prop.rotorInertiaKgM2 * pw(4.5)), estimate: true },
    motor: {
      ...f.motor, model: `${f.motor.model} — ölçeklenmiş`, kvRpmPerV: sig(f.motor.kvRpmPerV * pw(-0.75)), resistanceOhm: sig(f.motor.resistanceOhm * pw(-2.75)),
      noLoadCurrentA: sig(f.motor.noLoadCurrentA * pw(2.75)), maxCurrentA: sig(f.motor.maxCurrentA * pw(2.75)),
      thermalMassJPerK: sig(f.motor.thermalMassJPerK * pw(2.5)), thermalResistanceKPerW: sig(f.motor.thermalResistanceKPerW * pw(-2)), estimate: true,
    },
    esc: { ...f.esc, model: `${f.esc.model} — ölçeklenmiş`, maxContinuousCurrentA: sig(f.esc.maxContinuousCurrentA * pw(2.75)), resistanceOhm: sig(f.esc.resistanceOhm * pw(-2.75)), estimate: true },
    battery: { ...f.battery, model: `${f.battery.model} — ölçeklenmiş`, cellCapacityAh: sig(f.battery.cellCapacityAh * pw(2.5)), cellResistanceOhm: sig(f.battery.cellResistanceOhm * pw(-2.5)), estimate: true },
    hingeActuator: { ...f.hingeActuator, maxTorqueNm: sig(f.hingeActuator.maxTorqueNm * pw(3.5)), estimate: true },
    transition: {
      ...f.transition, openCompleteSpeedMs: sig(f.transition.openCompleteSpeedMs * pw(0.25)), fbwMinSpeedMs: sig(f.transition.fbwMinSpeedMs * pw(0.25)),
      assistSpeedMs: sig(f.transition.assistSpeedMs * pw(0.25)), estimate: true,
    },
    missionDefault: { ...f.missionDefault, cruiseSpeedMs: sig(f.missionDefault.cruiseSpeedMs * pw(0.25)) },
  };
}
