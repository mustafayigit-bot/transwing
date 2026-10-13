/**
 * Generates public/specs.json: the 7 m reference product (T7-C), its military variant and
 * the scaled 5 m / 10 m family members. ALL NUMBERS ARE PLACEHOLDERS (estimate: true).
 *
 * Run: npm run gen:specs   (after that, edit public/specs.json directly with real data;
 * the generator is only a bootstrap and overwrites the file.)
 */
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { designHingeForSweep, exposedSemiSpanM, foldPoint, rotorHubCruise } from '../src/kinematics/hinge';
import { massProperties } from '../src/physics/massProps';
import { scaleProduct } from '../src/data/scaling';
import { validateSpecs, type Part, type PayloadOption, type Product, type Scenario, type SpecsFile } from '../src/data/schema';

// Hover configuration: wing span swept 65° aft of lateral, thrust axis vertical.
const hinge = designHingeForSweep(65);
const r = (x: number, n = 3) => Math.round(x * 10 ** n) / 10 ** n;

function referenceT7(): Product {
  const spanM = 7.0;
  const chordRootM = 0.75;
  const chordTipM = 0.45;
  const L = 3.8;
  const D = 0.42;
  const hingePos: [number, number, number] = [0.95, 0.15, 0.26];
  const rotors = [
    { wing: 'L' as const, spanFraction: 0.32, diameterM: 1.3, foldableInCruise: false, hubOffsetM: 0.45 },
    { wing: 'L' as const, spanFraction: 0.85, diameterM: 1.3, foldableInCruise: true, hubOffsetM: 0.45 },
    { wing: 'R' as const, spanFraction: 0.32, diameterM: 1.3, foldableInCruise: false, hubOffsetM: 0.45 },
    { wing: 'R' as const, spanFraction: 0.85, diameterM: 1.3, foldableInCruise: true, hubOffsetM: 0.45 },
  ];
  const product: Product = {
    id: 'T7-C',
    name: 'Specter T7 Sivil',
    sizeClass: '7 m',
    variant: 'civil',
    geometry: {
      spanM, chordRootM, chordTipM, airfoil: '4415',
      fuselageLengthM: L, fuselageDiameterM: D, landingGearHeightM: 0.36,
      hinge: { positionM: hingePos, axisAzimuthDeg: r(hinge.axisAzimuthDeg, 2), axisElevationDeg: r(hinge.axisElevationDeg, 2), maxFoldDeg: r(hinge.maxFoldDeg, 2), estimate: true },
      rotors,
      estimate: true,
    },
    mass: { emptyKg: 0, batteryKg: 40, payloadMaxKg: 20, estimate: true },
    propulsion: { motorMaxPowerW: 16000, figureOfMerit: 0.72, propEfficiency: 0.8, motorEscEfficiency: 0.88, estimate: true },
    energy: { batteryWh: 7600, usableFraction: 0.85, estimate: true },
    aero: { cd0: 0.032, oswaldE: 0.8, clMax: 1.4, sideCd: 1.1, estimate: true },
    parts: [],
    payloadOptions: [],
  };
  const semi = exposedSemiSpanM(product);
  const cMean = (chordRootM + chordTipM) / 2;
  const wingPos = (side: 'L' | 'R'): [number, number, number] => [
    r(hingePos[0] - 0.15 * cMean), hingePos[1], r((side === 'R' ? 1 : -1) * (hingePos[2] + 0.42 * semi)),
  ];
  const part = (p: Omit<Part, 'estimate' | 'material' | 'bomRef' | 'attach'> & Partial<Part>): Part => ({
    attach: 'body', material: '—', bomRef: '—', estimate: true, ...p,
  });
  const parts: Part[] = [
    part({ id: 'fuselage', name: 'Gövde', group: 'Airframe', kind: 'fuselage', massKg: 19.5, positionM: [0, 0, 0], material: 'Karbon/epoksi sandviç', bomRef: 'AF-100' }),
    part({ id: 'nose_cone', name: 'Burun konisi', group: 'Airframe', kind: 'nose', massKg: 2, positionM: [r(L / 2 - 0.2), 0, 0], material: 'Cam elyaf', bomRef: 'AF-110' }),
    part({ id: 'harness', name: 'Kablo demeti', group: 'Airframe', kind: 'box', massKg: 3, positionM: [0.2, 0.05, 0], sizeM: [1.4, 0.06, 0.08], material: 'Bakır/PTFE', bomRef: 'EL-300' }),
    part({ id: 'tail', name: 'Kuyruk / stabilizör', group: 'Airframe', kind: 'tail', massKg: 4, positionM: [r(-L / 2 + 0.25), 0.2, 0], material: 'Karbon/köpük', bomRef: 'AF-130' }),
    part({ id: 'wing_left', name: 'Sol kanat', group: 'Wing', kind: 'wing', massKg: 14, positionM: wingPos('L'), attach: 'wingL', material: 'Karbon kiriş + köpük', bomRef: 'WG-200L' }),
    part({ id: 'wing_right', name: 'Sağ kanat', group: 'Wing', kind: 'wing', massKg: 14, positionM: wingPos('R'), attach: 'wingR', material: 'Karbon kiriş + köpük', bomRef: 'WG-200R' }),
    part({ id: 'hinge_left', name: 'Sol menteşe + aktüatör', group: 'Wing', kind: 'hinge', massKg: 2.5, positionM: [hingePos[0], hingePos[1], -hingePos[2]], material: 'Ti-6Al-4V / Al 7075', bomRef: 'WG-210L' }),
    part({ id: 'hinge_right', name: 'Sağ menteşe + aktüatör', group: 'Wing', kind: 'hinge', massKg: 2.5, positionM: hingePos, material: 'Ti-6Al-4V / Al 7075', bomRef: 'WG-210R' }),
  ];
  rotors.forEach((rot, i) => {
    const hub = rotorHubCruise(product, rot);
    const tag = `${rot.wing}${i % 2 === 0 ? 1 : 2}`;
    const attach = rot.wing === 'L' ? 'wingL' : 'wingR';
    parts.push(part({ id: `nacelle_${tag}`, name: `Nasel + motor ${tag}`, group: 'Propulsion', kind: 'nacelle', massKg: 4.5, positionM: [r(hub.x - rot.hubOffsetM * 0.55), r(hub.y), r(hub.z)], attach, rotorIndex: i, material: 'BLDC motor / CFRP nasel', bomRef: `PR-40${i}` }));
    parts.push(part({ id: `prop_${tag}`, name: `Pervane ${tag}${rot.foldableInCruise ? ' (katlanır)' : ''}`, group: 'Propulsion', kind: 'rotor', massKg: 1, positionM: [r(hub.x), r(hub.y), r(hub.z)], attach, rotorIndex: i, material: 'CFRP', bomRef: `PR-41${i}` }));
  });
  parts.push(
    part({ id: 'avionics', name: 'Aviyonik kutusu (FCC, INS)', group: 'Avionics', kind: 'avionics', massKg: 5, positionM: [0.55, 0.04, 0], sizeM: [0.36, 0.16, 0.22], material: 'Al 6061 kutu', bomRef: 'AV-500' }),
    part({ id: 'datalink', name: 'Veri linki + anten', group: 'Avionics', kind: 'box', massKg: 1.5, positionM: [-1.1, 0.26, 0], sizeM: [0.2, 0.1, 0.06], material: '—', bomRef: 'AV-510' }),
    part({ id: 'battery', name: 'Batarya paketi', group: 'Energy', kind: 'battery', massKg: 40, positionM: [0, -0.02, 0], sizeM: [0.75, 0.22, 0.26], material: 'Li-ion 21700', bomRef: 'EN-600' }),
    part({ id: 'payload', name: 'Payload modülü', group: 'Payload', kind: 'payload', massKg: 0, positionM: [-0.7, -0.3, 0], material: 'değişken', bomRef: 'PL-700' }),
    part({ id: 'landing_gear', name: 'İniş takımı (kızak)', group: 'Landing', kind: 'gear', massKg: 5, positionM: [-0.3, -0.4, 0], material: 'Karbon tüp', bomRef: 'LG-800' }),
  );
  product.parts = parts;
  product.payloadOptions = civilPayloads();
  return product;
}

function civilPayloads(): PayloadOption[] {
  return [
    { id: 'eo_gimbal', name: 'EO kamera gimbal', massKg: 4, dragAreaM2: 0.015, variant: 'civil', shape: 'gimbal', estimate: true },
    { id: 'lidar', name: 'LiDAR', massKg: 6, dragAreaM2: 0.02, variant: 'civil', shape: 'lidar', estimate: true },
    { id: 'cargo', name: 'Kargo kutusu', massKg: 20, dragAreaM2: 0.05, variant: 'civil', shape: 'box', estimate: true },
    { id: 'sar_kit', name: 'Arama-kurtarma paketi', massKg: 12, dragAreaM2: 0.035, variant: 'civil', shape: 'pod', estimate: true },
  ];
}

function militaryPayloads(): PayloadOption[] {
  return [
    { id: 'eoir_gimbal', name: 'EO/IR gimbal', massKg: 8, dragAreaM2: 0.02, variant: 'military', shape: 'gimbal', estimate: true },
    { id: 'sigint', name: 'SIGINT modülü', massKg: 10, dragAreaM2: 0.03, variant: 'military', shape: 'pod', estimate: true },
    { id: 'comms_relay', name: 'Haberleşme röle modülü', massKg: 9, dragAreaM2: 0.035, variant: 'military', shape: 'antenna', estimate: true },
    { id: 'cargo', name: 'Kargo', massKg: 20, dragAreaM2: 0.05, variant: 'military', shape: 'box', estimate: true },
  ];
}

function militarise(civil: Product): Product {
  const heavier: Record<string, number> = { avionics: 2.2, datalink: 2.5, fuselage: 1.1, harness: 1.3 };
  return {
    ...structuredClone(civil),
    id: civil.id.replace('-C', '-M'),
    name: civil.name.replace('Sivil', 'Askeri'),
    variant: 'military',
    parts: civil.parts.map((p) => ({ ...p, massKg: r(p.massKg * (heavier[p.id] ?? 1), 2), material: p.id === 'avionics' ? 'MIL-STD-810 kutu' : p.material })),
    payloadOptions: militaryPayloads(),
  };
}

/** Place the battery so the hover CG (half payload) sits on the rotor thrust centroid; finalise emptyKg. */
function balance(p: Product): Product {
  const fold = p.geometry.hinge.maxFoldDeg;
  const bat = p.parts.find((x) => x.kind === 'battery')!;
  const halfPayload: PayloadOption = { id: '_', name: '_', massKg: p.mass.payloadMaxKg / 2, dragAreaM2: 0, variant: 'both', shape: 'box', estimate: true };
  // With the battery at x = 0 it contributes nothing to the x-moment, so the required
  // battery station follows directly from the remaining first moment.
  bat.positionM = [0, bat.positionM[1], 0];
  const mp0 = massProperties(p, halfPayload, fold).value;
  const hoverHubs = p.geometry.rotors.map((rot) => foldPoint(p.geometry.hinge, rot.wing, fold, rotorHubCruise(p, rot)));
  const xt = hoverHubs.reduce((s, h) => s + h.x, 0) / hoverHubs.length;
  const halfL = p.geometry.fuselageLengthM / 2 - (bat.sizeM?.[0] ?? 0.5) / 2 - 0.05;
  const xb = Math.max(-halfL, Math.min(halfL, ((xt - mp0.cgM[0]) * mp0.totalKg) / bat.massKg));
  bat.positionM = [r(xb), bat.positionM[1], 0];
  p.mass.emptyKg = r(p.parts.filter((x) => x.group !== 'Payload' && x.kind !== 'battery').reduce((s, x) => s + x.massKg, 0), 2);
  p.mass.batteryKg = bat.massKg;
  return p;
}

function scenarios(): Scenario[] {
  const sc = (s: Omit<Scenario, 'sim'> & { sim?: Partial<Scenario['sim']> }): Scenario => ({
    ...s,
    sim: { durationS: 30, seed: 42, gustStartS: 5, gustDurationS: 2, ...s.sim },
  });
  return [
    sc({ id: 'calm_sl', name: 'Sakin deniz seviyesi', description: 'ISA, 0 m, rüzgârsız hover.', env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 0, windFromDeg: 0, turbulence: 'none', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'hover', cruiseMotors: 2, batterySoc: 1 } }),
    sc({ id: 'crosswind_15', name: '15 m/s yan rüzgârda hover', description: 'Sağdan 15 m/s sabit rüzgâr, hafif türbülans.', env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 15, windFromDeg: 90, turbulence: 'light', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'hover', cruiseMotors: 2, batterySoc: 1 } }),
    sc({ id: 'hot_high', name: 'Sıcak ve yüksek (2000 m, ISA+25)', description: 'Düşük hava yoğunluğu: hover güç marjı azalır.', env: { altitudeM: 2000, isaDeltaK: 25, windSpeedMs: 5, windFromDeg: 45, turbulence: 'light', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'hover', cruiseMotors: 2, batterySoc: 1 } }),
    sc({ id: 'gust_transition', name: 'Geçişte gust', description: 'Hover→cruise geçişinde 8 m/s 1-cosine gust.', env: { altitudeM: 100, isaDeltaK: 0, windSpeedMs: 4, windFromDeg: 0, turbulence: 'light', gustAmplitudeMs: 8, logProfile: true }, config: { flightMode: 'transition', cruiseMotors: 4, batterySoc: 1 }, sim: { durationS: 25, gustStartS: 8, gustDurationS: 2.5 } }),
    sc({ id: 'severe_cruise', name: 'Şiddetli türbülansta cruise', description: 'MIL-HDBK-1797 şiddetli Dryden türbülansı, 2 motor cruise.', env: { altitudeM: 300, isaDeltaK: 0, windSpeedMs: 10, windFromDeg: 30, turbulence: 'severe', gustAmplitudeMs: 0, logProfile: true }, config: { flightMode: 'cruise', cruiseMotors: 2, batterySoc: 1 }, sim: { durationS: 40 } }),
    sc({ id: 'ship_deck', name: 'Gemi güvertesine iniş (sabit rüzgâr + türbülans)', description: '12 m/s karşıdan rüzgâr + orta türbülans + gust, 15 m hover.', env: { altitudeM: 15, isaDeltaK: 5, windSpeedMs: 12, windFromDeg: 20, turbulence: 'moderate', gustAmplitudeMs: 5, logProfile: false }, config: { flightMode: 'hover', cruiseMotors: 2, batterySoc: 0.6 }, sim: { durationS: 40, gustStartS: 12, gustDurationS: 3 } }),
  ];
}

const t7c = balance(referenceT7());
const t7m = balance(militarise(t7c));
const family: Product[] = [];
for (const [s, cls] of [[5 / 7, '5 m'], [1, '7 m'], [10 / 7, '10 m']] as const) {
  const tag = cls.replace(' m', '');
  if (s === 1) {
    family.push(t7c, t7m);
  } else {
    family.push(balance(scaleProduct(t7c, s, `T${tag}-C`, `Specter T${tag} Sivil`, cls)));
    family.push(balance(scaleProduct(t7m, s, `T${tag}-M`, `Specter T${tag} Askeri`, cls)));
  }
}

const specs: SpecsFile = {
  version: 1,
  title: 'Specter Transwing VTOL ürün ailesi — YER TUTUCU VERİ',
  referenceProductId: 'T7-C',
  mission: { takeoffHoverS: 60, landingHoverS: 60, transitionS: 20, reserveFraction: 0.2, controlReserveFraction: 0.15, hoverHeightM: 30 },
  scenarios: scenarios(),
  products: family,
};

const { issues } = validateSpecs(specs);
for (const i of issues) console.warn(`[${i.level}] ${i.productId ?? ''} ${i.message}`);
const out = resolve(process.cwd(), 'public/specs.json');
writeFileSync(out, JSON.stringify(specs, null, 2) + '\n');
console.log(`Wrote ${out} (${family.length} products, ${specs.scenarios.length} scenarios)`);
