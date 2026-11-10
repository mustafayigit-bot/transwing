/**
 * Generates public/specs.json.
 *
 * Reference product: Transwing 4.0 (TW4) — 4.0 m span, 40 kg MTOW, values supplied by the design team
 * (estimate: false) plus clearly flagged assumptions (estimate: true) where data is still missing.
 * The 5 m / 7 m / 10 m family (civil + military) is derived from TW4 with the scaling laws.
 *
 * Run: npm run gen:specs   (overwrites the file — edit this script or the JSON, not both).
 */
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { designHingeForSweep, exposedSemiSpanM, foldPoint, rotorHubCruise } from '../src/kinematics/hinge';
import { massProperties } from '../src/physics/massProps';
import { scaleProduct } from '../src/data/scaling';
import { validateSpecs, type Part, type Product, type Scenario, type SpecsFile } from '../src/data/schema';
import { analyze } from '../src/physics/fdm/trim';

// ASSUMPTION (until CAD, Faz 3): hover configuration = folding panel swept 65° aft, thrust axis vertical.
const hinge = designHingeForSweep(65);
const r = (x: number, n = 3) => Math.round(x * 10 ** n) / 10 ** n;

/** Generic propeller map for a 28×13" folding VTOL prop (replace with test data). */
function propTable(ct0: number, cp0: number, j0: number, j0p: number) {
  const j: number[] = [], ct: number[] = [], cp: number[] = [];
  for (let x = 0; x <= 1.0001; x += 0.05) {
    j.push(r(x, 2));
    ct.push(r(Math.max(-0.02, ct0 * (1 - (x / j0) ** 2)), 5));
    cp.push(r(Math.max(0.0015, cp0 * (1 - (x / j0p) ** 3)), 5));
  }
  return { j, ct, cp };
}

function transwing4(): Product {
  const spanM = 4.0, chordRootM = 0.34, chordTipM = 0.212;
  const L = 2.0, D = 0.3;
  const xQc = 0.375; // wing ¼ MAC is 0.625 m behind the nose (nose at x = +1.0)
  // ASSUMPTION (until CAD): low-wing layout — the propeller thrust line passes close to the vertical CG.
  // With a high wing the thrust line sits 0.15–0.35 m above the CG and the forward transition cannot be
  // trimmed in pitch by differential thrust (see README, "Geçiş koridoru").
  const hingePos: [number, number, number] = [xQc + 0.08, -0.06, 0.499];
  // nacelles at y = 0.529 m and 1.290 m; span fraction is measured from the hinge over the folding panel
  const semi = spanM / 2 - hingePos[2];
  const fr = (y: number) => r((y - hingePos[2]) / semi, 4);
  const rotors = [
    { wing: 'L' as const, spanFraction: fr(0.529), diameterM: 0.7112, foldableInCruise: false, hubOffsetM: 0.3 },
    { wing: 'L' as const, spanFraction: fr(1.29), diameterM: 0.7112, foldableInCruise: true, hubOffsetM: 0.3 },
    { wing: 'R' as const, spanFraction: fr(0.529), diameterM: 0.7112, foldableInCruise: false, hubOffsetM: 0.3 },
    { wing: 'R' as const, spanFraction: fr(1.29), diameterM: 0.7112, foldableInCruise: true, hubOffsetM: 0.3 },
  ];
  const pt = propTable(0.105, 0.036, 0.72, 0.8);
  const product: Product = {
    id: 'TW4',
    name: 'Transwing 4.0',
    sizeClass: '4 m',
    variant: 'civil',
    geometry: {
      spanM, chordRootM, chordTipM, airfoil: '4412',
      fuselageLengthM: L, fuselageDiameterM: D, landingGearHeightM: 0.25,
      wingAreaM2: 1.125, wingQcXM: xQc, incidenceDeg: 2.5,
      aileron: { startYM: 1.15, endYM: 1.82, chordFraction: 0.25, maxDeflectionDeg: 25 },
      tail: { type: 'v', areaM2: 0.2, dihedralDeg: 38, quarterChordXM: xQc - 1.35, rootYM: 0.1, panelSpanM: 0.42, controlChordFraction: 0.3, maxDeflectionDeg: 25 },
      hinge: { positionM: hingePos, axisAzimuthDeg: r(hinge.axisAzimuthDeg, 2), axisElevationDeg: r(hinge.axisElevationDeg, 2), maxFoldDeg: r(hinge.maxFoldDeg, 2), estimate: true },
      rotors,
      estimate: false,
    },
    mass: { emptyKg: 19.5, batteryKg: 10, payloadMaxKg: 10.5, estimate: false },
    // legacy quick-look model; the 6-DOF model below is authoritative
    propulsion: { motorMaxPowerW: 3700, figureOfMerit: 0.75, propEfficiency: 0.62, motorEscEfficiency: 0.9, fourMotorEtaFactor: 0.67, defaultCruiseMotors: 4, estimate: true },
    energy: { batteryWh: 2900.8, usableFraction: 0.85, estimate: false },
    // CD0 and e reproduce the team's (L/D)max = 18.07 at CL = 1.014; CLmax from Vs = 20.2 m/s at 40 kg
    aero: { cd0: 0.0281, oswaldE: 0.82, clMax: 1.4, sideCd: 1.1, estimate: false },
    parts: [],
    payloadOptions: [
      { id: 'mtow', name: 'MTOW: EO/IR + ek görev yükü', massKg: 10.5, dragAreaM2: 0.004, variant: 'both', shape: 'gimbal', estimate: false },
      { id: 'eoir', name: 'EO/IR çift sensörlü gimbal', massKg: 2.5, dragAreaM2: 0.003, variant: 'both', shape: 'gimbal', estimate: false },
      { id: 'eoir_plus', name: 'EO/IR + opsiyonel sensör', massKg: 8.5, dragAreaM2: 0.004, variant: 'both', shape: 'gimbal', estimate: false },
    ],
    fdm: {
      prop: { model: 'Lighter GFFA 28×13-T (genel pervane haritası — test verisi gerekli)', pitchM: 0.3302, rotorInertiaKgM2: 0.011, ...pt, estimate: true },
      motor: { model: 'Hacker Q80-9M V2 (160 KV)', kvRpmPerV: 160, resistanceOhm: 0.018, noLoadCurrentA: 1.6, maxCurrentA: 180, thermalMassJPerK: 450, thermalResistanceKPerW: 0.3, maxTempC: 120, estimate: true },
      esc: { model: 'CE1767 14S 150 A', maxContinuousCurrentA: 150, resistanceOhm: 0.003, maxTempC: 90, estimate: true },
      battery: {
        model: 'MXARI LP12048120N14 — 14S4P', cellsS: 14, cellsP: 4, cellCapacityAh: 14, cellNominalV: 3.7, cellFullV: 4.2, cellCutoffV: 3.0,
        cellResistanceOhm: 0.003, maxContinuousC: 3, peakC: 5,
        ocvSoc: [0, 0.03, 0.07, 0.12, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1],
        ocvV: [3.0, 3.2, 3.36, 3.46, 3.55, 3.62, 3.67, 3.73, 3.81, 3.9, 3.99, 4.09, 4.2],
        reserveSoc: 0.15, lowCellV: 3.3, criticalCellV: 3.1, armCellV: 4.0, estimate: false,
      },
      hingeActuator: { type: 'BLDC + planet redüktör / bilyalı vida, mutlak enkoder', rateDegS: r(hinge.maxFoldDeg / 4, 1), maxTorqueNm: 35, estimate: false },
      section: { liftSlopePerRad: 6.0, alphaZeroLiftDeg: -4.0, clMax: 1.5, cd0: 0.009, cm0: -0.09, cd90: 1.3 },
      targetMaxLD: 18.07,
      transition: { openCompleteSpeedMs: 20.5, fbwMinSpeedMs: 23.5, assistSpeedMs: 20.5, initialOpenFraction: 0.35, estimate: true },
      missionDefault: { hoverHeightM: 20, cruiseAltitudeM: 200, cruiseSpeedMs: 23.7, takeoffHoverS: 180, landingHoverS: 180, waypointsM: [[38000, 0]] },
    },
  };
  const cMean = (chordRootM + chordTipM) / 2;
  const semiE = exposedSemiSpanM(product);
  const part = (p: Omit<Part, 'estimate' | 'material' | 'bomRef' | 'attach'> & Partial<Part>): Part => ({ attach: 'body', material: '—', bomRef: '—', estimate: true, ...p });
  const wingPos = (side: 1 | -1): [number, number, number] => [r(xQc - 0.15 * cMean), hingePos[1], r(side * (hingePos[2] + 0.42 * semiE))];
  // Group totals are team data: structure 10.03 kg, propulsion 6.55 kg, avionics/actuators 2.92 kg.
  // The split inside each group and all positions are estimates until the CAD package (Faz 3).
  const parts: Part[] = [
    part({ id: 'fuselage', name: 'Gövde', group: 'Airframe', kind: 'fuselage', massKg: 3.7, positionM: [-0.05, 0, 0], material: 'Karbon/epoksi kompozit', bomRef: 'AF-100' }),
    part({ id: 'nose_cone', name: 'Burun konisi', group: 'Airframe', kind: 'nose', massKg: 0.3, positionM: [r(L / 2 - 0.1), 0, 0], material: 'Cam elyaf (RF geçirgen)', bomRef: 'AF-110' }),
    part({ id: 'center_wing', name: 'Orta kanat (sabit geçiş kutusu)', group: 'Wing', kind: 'centerwing', massKg: 1.2, positionM: [r(xQc - 0.05), hingePos[1], 0], material: 'Karbon kiriş + sandviç', bomRef: 'WG-190' }),
    part({ id: 'tail', name: 'V-kuyruk (ruddervator)', group: 'Airframe', kind: 'tail', massKg: 0.73, positionM: [r(xQc - 1.35 - 0.03), 0.22, 0], material: 'Karbon/köpük', bomRef: 'AF-130' }),
    part({ id: 'wing_left', name: 'Sol dış kanat paneli', group: 'Wing', kind: 'wing', massKg: 1.4, positionM: wingPos(-1), attach: 'wingL', material: 'Karbon kiriş + köpük', bomRef: 'WG-200L' }),
    part({ id: 'wing_right', name: 'Sağ dış kanat paneli', group: 'Wing', kind: 'wing', massKg: 1.4, positionM: wingPos(1), attach: 'wingR', material: 'Karbon kiriş + köpük', bomRef: 'WG-200R' }),
    part({ id: 'hinge_left', name: 'Sol menteşe + eyleyici', group: 'Wing', kind: 'hinge', massKg: 0.55, positionM: [hingePos[0], hingePos[1], -hingePos[2]], material: 'Ti-6Al-4V / Al 7075', bomRef: 'WG-210L' }),
    part({ id: 'hinge_right', name: 'Sağ menteşe + eyleyici', group: 'Wing', kind: 'hinge', massKg: 0.55, positionM: hingePos, material: 'Ti-6Al-4V / Al 7075', bomRef: 'WG-210R' }),
  ];
  rotors.forEach((rot, i) => {
    const hub = rotorHubCruise(product, rot);
    const tag = `${rot.wing}${i % 2 === 0 ? 1 : 2}`;
    const attach = rot.wing === 'L' ? 'wingL' : 'wingR';
    parts.push(part({ id: `nacelle_${tag}`, name: `Nasel + motor + ESC ${tag}`, group: 'Propulsion', kind: 'nacelle', massKg: 1.25, positionM: [r(hub.x - rot.hubOffsetM * 0.45), r(hub.y), r(hub.z)], attach, rotorIndex: i, material: 'Hacker Q80-9M V2 / CE1767', bomRef: `PR-40${i}` }));
    parts.push(part({ id: `prop_${tag}`, name: `Pervane ${tag} (28×13 katlanır)`, group: 'Propulsion', kind: 'rotor', massKg: 0.22, positionM: [r(hub.x), r(hub.y), r(hub.z)], attach, rotorIndex: i, material: 'Karbon (Lighter GFFA)', bomRef: `PR-41${i}` }));
  });
  parts.push(
    part({ id: 'harness', name: 'Güç kablajı', group: 'Propulsion', kind: 'box', massKg: 0.67, positionM: [0.2, 0.05, 0], sizeM: [0.9, 0.03, 0.04], material: 'Bakır/silikon', bomRef: 'EL-300' }),
    part({ id: 'avionics', name: 'Aviyonik (Cube Orange, GPS, PDB)', group: 'Avionics', kind: 'avionics', massKg: 1.22, positionM: [0.62, 0.02, 0], sizeM: [0.22, 0.09, 0.14], material: 'ArduPilot Plane 4.7', bomRef: 'AV-500' }),
    part({ id: 'servos', name: 'Kontrol yüzeyi servoları (×4)', group: 'Avionics', kind: 'box', massKg: 0.3, positionM: [-0.3, 0.08, 0], sizeM: [0.08, 0.03, 0.2], material: '—', bomRef: 'AV-520' }),
    part({ id: 'datalink', name: 'Veri linki + anten', group: 'Avionics', kind: 'box', massKg: 0.3, positionM: [-0.45, 0.16, 0], sizeM: [0.1, 0.04, 0.05], material: '—', bomRef: 'AV-510' }),
    part({ id: 'battery', name: 'Batarya paketi 14S4P', group: 'Energy', kind: 'battery', massKg: 10, positionM: [0.2, 0.02, 0], sizeM: [0.5, 0.14, 0.2], material: 'MXARI LP12048120N14 (NCM)', bomRef: 'EN-600', estimate: false }),
    part({ id: 'payload', name: 'Payload modülü', group: 'Payload', kind: 'payload', massKg: 0, positionM: [0.7, -0.19, 0], material: 'değişken', bomRef: 'PL-700' }),
    part({ id: 'landing_gear', name: 'İniş takımı (kızak)', group: 'Landing', kind: 'gear', massKg: 1.3, positionM: [0.1, -0.3, 0], material: 'Karbon tüp', bomRef: 'LG-800' }),
  );
  product.parts = parts;
  return product;
}

function militarise(civil: Product): Product {
  const heavier: Record<string, number> = { avionics: 2.2, datalink: 2.5, fuselage: 1.1, harness: 1.3 };
  const p: Product = {
    ...structuredClone(civil),
    id: civil.id.replace('-C', '-M'),
    name: civil.name.replace('Sivil', 'Askeri'),
    variant: 'military',
  };
  p.parts = civil.parts.map((x) => ({ ...x, massKg: r(x.massKg * (heavier[x.id] ?? 1), 3), material: x.id === 'avionics' ? 'MIL-STD-810 kutu' : x.material }));
  p.payloadOptions = [
    { ...civil.payloadOptions[0], id: 'mtow', name: 'MTOW: EO/IR + SIGINT / röle', variant: 'military' },
    { ...civil.payloadOptions[1], id: 'eoir', name: 'EO/IR gimbal', variant: 'military' },
    { ...civil.payloadOptions[2], id: 'sigint', name: 'SIGINT modülü', shape: 'pod', variant: 'military' },
    { ...civil.payloadOptions[2], id: 'comms_relay', name: 'Haberleşme röle modülü', shape: 'antenna', variant: 'military' },
  ];
  return p;
}

/**
 * Place the battery so that the cruise static margin is ≈ 10 % MAC (neutral point from the 6-DOF
 * model), then report the hover balance. Cruise stability takes priority; the hover CG offset is
 * handled by differential thrust.
 */
function balance(p: Product): Product {
  const bat = p.parts.find((x) => x.kind === 'battery')!;
  const halfL = p.geometry.fuselageLengthM / 2 - (bat.sizeM?.[0] ?? 0.5) / 2 - 0.05;
  const mtow = p.payloadOptions[0];
  for (let pass = 0; pass < 3; pass++) {
    const rep = analyze(p, mtow, 0, 0);
    const d = rep.derivatives;
    const mac = Math.abs(d.staticMargin) > 1e-4 ? (d.cgXM - d.neutralPointXM) / d.staticMargin : p.geometry.chordRootM * 0.83;
    const target = d.neutralPointXM + 0.1 * mac; // CG 10 % MAC ahead of the neutral point
    const dx = target - rep.cgCruise[0];
    const xb = Math.max(-halfL, Math.min(halfL, bat.positionM[0] + (dx * rep.massKg) / bat.massKg));
    bat.positionM = [r(xb), bat.positionM[1], 0];
  }
  p.mass.emptyKg = r(p.parts.filter((x) => x.group !== 'Payload' && x.kind !== 'battery').reduce((s, x) => s + x.massKg, 0), 2);
  p.mass.batteryKg = bat.massKg;
  return p;
}

function scenarios(): Scenario[] {
  const sc = (s: Omit<Scenario, 'sim'> & { sim?: Partial<Scenario['sim']> }): Scenario => ({ ...s, sim: { durationS: 30, seed: 42, gustStartS: 5, gustDurationS: 2, ...s.sim } });
  return [
    sc({ id: 'calm_sl', name: 'Sakin deniz seviyesi', description: 'ISA, 0 m, rüzgârsız hover.', env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 0, windFromDeg: 0, turbulence: 'none', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'hover', cruiseMotors: 4, batterySoc: 1 } }),
    sc({ id: 'crosswind_15', name: '15 m/s yan rüzgârda hover', description: 'Sağdan 15 m/s sabit rüzgâr, hafif türbülans.', env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 15, windFromDeg: 90, turbulence: 'light', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'hover', cruiseMotors: 4, batterySoc: 1 } }),
    sc({ id: 'hot_high', name: 'Sıcak ve yüksek (2000 m, ISA+25)', description: 'Düşük hava yoğunluğu: hover güç marjı azalır.', env: { altitudeM: 2000, isaDeltaK: 25, windSpeedMs: 5, windFromDeg: 45, turbulence: 'light', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'hover', cruiseMotors: 4, batterySoc: 1 } }),
    sc({ id: 'gust_transition', name: 'Geçişte gust', description: 'Hover→cruise geçişinde 8 m/s 1-cosine gust.', env: { altitudeM: 100, isaDeltaK: 0, windSpeedMs: 4, windFromDeg: 0, turbulence: 'light', gustAmplitudeMs: 8, logProfile: true }, config: { flightMode: 'transition', cruiseMotors: 4, batterySoc: 1 }, sim: { durationS: 25, gustStartS: 8, gustDurationS: 2.5 } }),
    sc({ id: 'severe_cruise', name: 'Şiddetli türbülansta cruise', description: 'MIL-HDBK-1797 şiddetli Dryden türbülansı.', env: { altitudeM: 300, isaDeltaK: 0, windSpeedMs: 10, windFromDeg: 30, turbulence: 'severe', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'cruise', cruiseMotors: 4, batterySoc: 1 }, sim: { durationS: 40 } }),
    sc({ id: 'ship_deck', name: 'Gemi güvertesine iniş (sabit rüzgâr + türbülans)', description: '12 m/s karşıdan rüzgâr + orta türbülans + gust, 15 m hover.', env: { altitudeM: 15, isaDeltaK: 5, windSpeedMs: 12, windFromDeg: 20, turbulence: 'moderate', gustAmplitudeMs: 5, logProfile: false }, config: { flightMode: 'hover', cruiseMotors: 4, batterySoc: 0.6 }, sim: { durationS: 40, gustStartS: 12, gustDurationS: 3 } }),
  ];
}

const tw4 = balance(transwing4());
const family: Product[] = [tw4];
for (const [span, cls] of [[5, '5 m'], [7, '7 m'], [10, '10 m']] as const) {
  const civil = scaleProduct(tw4, span / 4, `T${span}-C`, `Specter T${span} Sivil`, cls);
  family.push(balance(civil));
  const mil = militarise(civil);
  mil.name = `Specter T${span} Askeri`;
  family.push(balance(mil));
}

const specs: SpecsFile = {
  version: 2,
  title: 'Specter / Transwing VTOL ürün ailesi — referans: Transwing 4.0',
  referenceProductId: 'TW4',
  // 15 % SoC reserve is expressed through energy.usableFraction = 0.85; no additional reserve on top
  mission: { takeoffHoverS: 180, landingHoverS: 180, transitionS: 12, reserveFraction: 0, controlReserveFraction: 0.15, hoverHeightM: 20 },
  scenarios: scenarios(),
  products: family,
};

const { issues } = validateSpecs(specs);
for (const i of issues) console.warn(`[${i.level}] ${i.productId ?? ''} ${i.message}`);
const out = resolve(process.cwd(), 'public/specs.json');
writeFileSync(out, JSON.stringify(specs, null, 1) + '\n');
console.log(`Wrote ${out} (${family.length} products, ${specs.scenarios.length} scenarios)`);
void foldPoint; void massProperties;
