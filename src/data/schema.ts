/**
 * Specter product data schema (single source of truth = public/specs.json).
 * All units SI. Field names carry unit suffixes (M, Kg, W, Wh, Deg, M2 ...).
 * `estimate: true` marks placeholder / derived values that must be replaced
 * with real engineering data.
 */
import { z } from 'zod';

const vec3 = z.tuple([z.number(), z.number(), z.number()]);

export const PART_GROUPS = ['Airframe', 'Wing', 'Propulsion', 'Energy', 'Avionics', 'Payload', 'Landing'] as const;
export const PART_KINDS = [
  'fuselage', 'nose', 'tail', 'wing', 'hinge', 'nacelle', 'rotor', 'battery', 'avionics', 'payload', 'gear', 'box',
] as const;

export const hingeSchema = z.object({
  /** Hinge pivot point of the RIGHT wing in body frame (x fwd, y up, z right). Left wing is mirrored (z -> -z). */
  positionM: vec3,
  /** Azimuth of the right-wing hinge axis in the horizontal plane, measured from aft (-x) towards outboard (+z). */
  axisAzimuthDeg: z.number(),
  /** Elevation of the right-wing hinge axis above the horizontal plane (negative = pointing down). */
  axisElevationDeg: z.number(),
  /** Fold angle (rotation about hinge axis) for full hover configuration. 0 = cruise. */
  maxFoldDeg: z.number().positive(),
  estimate: z.boolean().default(true),
});

export const rotorSchema = z.object({
  wing: z.enum(['L', 'R']),
  /** Position along the semi-span measured from the hinge (0 = root, 1 = tip). */
  spanFraction: z.number().min(0).max(1),
  diameterM: z.number().positive(),
  /** True = this motor is switched off in 2-motor cruise and its blades fold against the nacelle. */
  foldableInCruise: z.boolean(),
  /** Distance of the propeller plane ahead of the wing quarter-chord line (cruise frame). */
  hubOffsetM: z.number().default(0.35),
});

export const partSchema = z.object({
  id: z.string().min(1),
  name: z.string(),
  group: z.enum(PART_GROUPS),
  kind: z.enum(PART_KINDS).default('box'),
  massKg: z.number().nonnegative(),
  /** Part centre of mass in body frame, cruise configuration. */
  positionM: vec3,
  /** Optional bounding size used by procedural geometry for box-like parts. */
  sizeM: vec3.optional(),
  /** Which rigid body the part is attached to. Wing-attached parts move with the hinge. */
  attach: z.enum(['body', 'wingL', 'wingR']).default('body'),
  /** Index into geometry.rotors for nacelle / rotor parts. */
  rotorIndex: z.number().int().nonnegative().optional(),
  material: z.string().default('—'),
  bomRef: z.string().default('—'),
  estimate: z.boolean().default(true),
});

export const payloadOptionSchema = z.object({
  id: z.string(),
  name: z.string(),
  massKg: z.number().nonnegative(),
  /** Equivalent flat-plate drag area added in cruise (CD·S, m²). */
  dragAreaM2: z.number().nonnegative(),
  variant: z.enum(['civil', 'military', 'both']),
  shape: z.enum(['gimbal', 'lidar', 'box', 'pod', 'antenna', 'none']).default('box'),
  estimate: z.boolean().default(true),
});

export const productSchema = z.object({
  id: z.string(),
  name: z.string(),
  sizeClass: z.string(),
  variant: z.enum(['civil', 'military']),
  /** Optional GLB model. Node names must match parts[].id, hinges = hinge_left / hinge_right. */
  modelUrl: z.string().optional(),
  derivedFrom: z.string().optional(),
  scaleFactor: z.number().optional(),
  geometry: z.object({
    spanM: z.number().positive(),
    chordRootM: z.number().positive(),
    chordTipM: z.number().positive(),
    airfoil: z.string().regex(/^\d{4}$/, 'NACA 4 haneli olmalı (ör. 4415)'),
    fuselageLengthM: z.number().positive(),
    fuselageDiameterM: z.number().positive(),
    landingGearHeightM: z.number().positive().default(0.4),
    hinge: hingeSchema,
    rotors: z.array(rotorSchema).min(2),
    estimate: z.boolean().default(true),
  }),
  mass: z.object({
    emptyKg: z.number().positive(),
    batteryKg: z.number().positive(),
    payloadMaxKg: z.number().nonnegative(),
    estimate: z.boolean().default(true),
  }),
  propulsion: z.object({
    /** Maximum continuous shaft power per motor. */
    motorMaxPowerW: z.number().positive(),
    figureOfMerit: z.number().min(0.3).max(0.9),
    propEfficiency: z.number().min(0.3).max(0.95),
    motorEscEfficiency: z.number().min(0.5).max(1),
    estimate: z.boolean().default(true),
  }),
  energy: z.object({
    batteryWh: z.number().positive(),
    usableFraction: z.number().min(0.1).max(1),
    estimate: z.boolean().default(true),
  }),
  aero: z.object({
    cd0: z.number().positive(),
    oswaldE: z.number().min(0.3).max(1),
    clMax: z.number().positive(),
    /** Drag coefficient of the folded (hover) airframe based on projected area. */
    sideCd: z.number().positive(),
    estimate: z.boolean().default(true),
  }),
  parts: z.array(partSchema).min(1),
  payloadOptions: z.array(payloadOptionSchema),
});

export const scenarioSchema = z.object({
  id: z.string(),
  name: z.string(),
  description: z.string().default(''),
  env: z.object({
    altitudeM: z.number(),
    isaDeltaK: z.number(),
    windSpeedMs: z.number().nonnegative(),
    /** Direction the wind comes FROM, relative to the vehicle nose (0 = headwind, 90 = from the right). */
    windFromDeg: z.number(),
    turbulence: z.enum(['none', 'light', 'moderate', 'severe']),
    gustAmplitudeMs: z.number().nonnegative(),
    logProfile: z.boolean().default(true),
  }),
  config: z.object({
    flightMode: z.enum(['hover', 'transition', 'cruise']),
    cruiseMotors: z.union([z.literal(2), z.literal(4)]).default(2),
    batterySoc: z.number().min(0).max(1).default(1),
  }),
  sim: z.object({
    durationS: z.number().positive().default(30),
    seed: z.number().int().default(1),
    gustStartS: z.number().nonnegative().default(5),
    gustDurationS: z.number().positive().default(2),
  }),
});

export const specsFileSchema = z.object({
  version: z.number(),
  title: z.string(),
  referenceProductId: z.string(),
  mission: z.object({
    takeoffHoverS: z.number().nonnegative(),
    landingHoverS: z.number().nonnegative(),
    transitionS: z.number().nonnegative(),
    reserveFraction: z.number().min(0).max(0.9),
    /** Hover thrust that must remain available for control (fraction of max). */
    controlReserveFraction: z.number().min(0).max(0.9),
    hoverHeightM: z.number().positive(),
  }),
  scenarios: z.array(scenarioSchema).min(1),
  products: z.array(productSchema).min(1),
});

export type Vec3 = [number, number, number];
export type Hinge = z.infer<typeof hingeSchema>;
export type Rotor = z.infer<typeof rotorSchema>;
export type Part = z.infer<typeof partSchema>;
export type PartGroup = (typeof PART_GROUPS)[number];
export type PayloadOption = z.infer<typeof payloadOptionSchema>;
export type Product = z.infer<typeof productSchema>;
export type Scenario = z.infer<typeof scenarioSchema>;
export type SpecsFile = z.infer<typeof specsFileSchema>;
export type MissionSpec = SpecsFile['mission'];

export interface ValidationIssue {
  productId?: string;
  level: 'error' | 'warning';
  message: string;
}

/** Schema validation plus engineering consistency checks. */
export function validateSpecs(raw: unknown): { specs: SpecsFile | null; issues: ValidationIssue[] } {
  const parsed = specsFileSchema.safeParse(raw);
  if (!parsed.success) {
    return {
      specs: null,
      issues: parsed.error.issues.map((i) => ({ level: 'error', message: `${i.path.join('.')}: ${i.message}` })),
    };
  }
  const specs = parsed.data;
  const issues: ValidationIssue[] = [];
  const ids = new Set<string>();
  for (const p of specs.products) {
    if (ids.has(p.id)) issues.push({ productId: p.id, level: 'error', message: `Yinelenen ürün id: ${p.id}` });
    ids.add(p.id);
    const partIds = new Set<string>();
    for (const part of p.parts) {
      if (partIds.has(part.id)) issues.push({ productId: p.id, level: 'error', message: `Yinelenen parça id: ${part.id}` });
      partIds.add(part.id);
      if ((part.kind === 'rotor' || part.kind === 'nacelle') && (part.rotorIndex === undefined || part.rotorIndex >= p.geometry.rotors.length)) {
        issues.push({ productId: p.id, level: 'error', message: `${part.id}: geçersiz rotorIndex` });
      }
    }
    const structural = p.parts.filter((x) => x.group !== 'Payload' && x.kind !== 'battery').reduce((s, x) => s + x.massKg, 0);
    if (Math.abs(structural - p.mass.emptyKg) / p.mass.emptyKg > 0.02) {
      issues.push({
        productId: p.id,
        level: 'warning',
        message: `Parça kütleleri toplamı (${structural.toFixed(1)} kg) emptyKg (${p.mass.emptyKg} kg) ile %2'den fazla farklı.`,
      });
    }
    const bat = p.parts.filter((x) => x.kind === 'battery').reduce((s, x) => s + x.massKg, 0);
    if (Math.abs(bat - p.mass.batteryKg) > 0.01 * p.mass.batteryKg + 1e-6) {
      issues.push({ productId: p.id, level: 'warning', message: `Batarya parçası kütlesi (${bat.toFixed(1)} kg) batteryKg ile uyuşmuyor.` });
    }
    for (const opt of p.payloadOptions) {
      if (opt.massKg > p.mass.payloadMaxKg + 1e-9) {
        issues.push({ productId: p.id, level: 'warning', message: `Payload "${opt.name}" (${opt.massKg} kg) payloadMaxKg değerini aşıyor.` });
      }
    }
    if (p.geometry.chordTipM > p.geometry.chordRootM) {
      issues.push({ productId: p.id, level: 'warning', message: 'Uç veter kök veterden büyük (ters taper).' });
    }
  }
  if (!ids.has(specs.referenceProductId)) {
    issues.push({ level: 'error', message: `referenceProductId (${specs.referenceProductId}) ürün listesinde yok.` });
  }
  return { specs, issues };
}
