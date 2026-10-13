import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { validateSpecs, type Product, type SpecsFile } from '../data/schema';
import { scaleProduct } from '../data/scaling';
import { designHingeForSweep, foldPoint, rotorHub, spanDirection, thrustDirection } from '../kinematics/hinge';
import { atmosphere } from './atmosphere';
import { cruisePerformance } from './cruise';
import { hoverPerformance, hoverWindCheck, hoverWindLimit } from './hover';
import { massProperties, payloadById } from './massProps';
import { missionPerformance } from './mission';
import { idealHoverPowerW, leastNorm, maxStaticThrustN, hoverShaftPowerW } from './rotor';
import { simulate, type SimInput } from './sim';
import { DrydenTurbulence, drydenParams, oneMinusCosineGust, windAtHeight } from './wind';

const raw = JSON.parse(readFileSync(resolve(__dirname, '../../public/specs.json'), 'utf8'));
const { specs, issues } = validateSpecs(raw);
const S = specs as SpecsFile;
const prod = (id: string) => S.products.find((p) => p.id === id) as Product;
const t7 = prod('T7-C');

describe('specs.json', () => {
  it('validates without errors and has 6 products', () => {
    expect(issues.filter((i) => i.level === 'error')).toEqual([]);
    expect(S.products).toHaveLength(6);
    expect(S.scenarios.length).toBeGreaterThanOrEqual(6);
  });
  it('rejects malformed data', () => {
    const bad = structuredClone(raw);
    bad.products[0].geometry.airfoil = 'ABC';
    expect(validateSpecs(bad).specs).toBeNull();
  });
  it('scales mass with s^2.5 and lengths with s', () => {
    const p = scaleProduct(t7, 10 / 7, 'X', 'X', '10 m');
    expect(p.geometry.spanM).toBeCloseTo(10, 2);
    expect(p.mass.batteryKg / t7.mass.batteryKg).toBeCloseTo(Math.pow(10 / 7, 2.5), 2);
    expect(p.parts.every((x) => x.estimate)).toBe(true);
  });
});

describe('hinge kinematics', () => {
  const h = t7.geometry.hinge;
  it('fold 0 is identity', () => {
    const p = new Vector3(1, 2, 3);
    expect(foldPoint(h, 'R', 0, p).distanceTo(p)).toBeLessThan(1e-9);
  });
  it('full fold points thrust up and wing span aft for both wings', () => {
    for (const side of ['L', 'R'] as const) {
      const t = thrustDirection(h, side, h.maxFoldDeg);
      expect(t.y).toBeGreaterThan(0.99);
      const sp = spanDirection(h, side, h.maxFoldDeg);
      expect(sp.x).toBeLessThan(-0.85); // swept back along fuselage
      expect(Math.abs(sp.y)).toBeLessThan(0.02);
      expect(Math.sign(sp.z)).toBe(side === 'R' ? 1 : -1);
    }
  });
  it('left and right wings are mirror images', () => {
    for (const rotor of t7.geometry.rotors.filter((r) => r.wing === 'R')) {
      const mirror = t7.geometry.rotors.find((r) => r.wing === 'L' && r.spanFraction === rotor.spanFraction)!;
      const a = rotorHub(t7, rotor, 60), b = rotorHub(t7, mirror, 60);
      expect(a.x).toBeCloseTo(b.x, 9);
      expect(a.y).toBeCloseTo(b.y, 9);
      expect(a.z).toBeCloseTo(-b.z, 9);
    }
  });
  it('designHingeForSweep reproduces 90° fold axis (-1,-1,1)/√3 at 120°', () => {
    const d = designHingeForSweep(90);
    expect(d.maxFoldDeg).toBeCloseTo(120, 6);
    expect(d.axisAzimuthDeg).toBeCloseTo(45, 6);
    expect(d.axisElevationDeg).toBeCloseTo((-Math.asin(1 / Math.sqrt(3)) * 180) / Math.PI, 6);
  });
  it('folding moves the CG (aft in hover)', () => {
    const c = massProperties(t7, null, 0).value.cgM[0];
    const hv = massProperties(t7, null, t7.geometry.hinge.maxFoldDeg).value.cgM[0];
    expect(hv).toBeLessThan(c - 0.1);
  });
});

describe('atmosphere', () => {
  it('ISA sea level and 2000 m', () => {
    expect(atmosphere(0).value.densityKgM3).toBeCloseTo(1.225, 3);
    expect(atmosphere(2000).value.densityKgM3).toBeCloseTo(1.0066, 3);
    expect(atmosphere(11000).value.temperatureK).toBeCloseTo(216.65, 2);
  });
  it('hot day reduces density', () => {
    expect(atmosphere(2000, 25).value.densityKgM3).toBeLessThan(atmosphere(2000).value.densityKgM3 * 0.93);
  });
});

describe('rotor / hover', () => {
  it('momentum theory and inverse are consistent', () => {
    const A = 1.2, rho = 1.225;
    const T = maxStaticThrustN(10000, rho, A, 0.7);
    expect(hoverShaftPowerW(T, rho, A, 0.7)).toBeCloseTo(10000, 3);
    expect(idealHoverPowerW(100, 1.225, 1)).toBeCloseTo(Math.pow(100, 1.5) / Math.sqrt(2.45), 9);
  });
  it('least norm satisfies constraints', () => {
    const A = [[1, 1, 1, 1], [1, -1, 1, -1], [2, 2, -1, -1]];
    const x = leastNorm(A, [10, 0.5, 1]);
    A.forEach((row, i) => expect(row.reduce((s, a, k) => s + a * x[k], 0)).toBeCloseTo([10, 0.5, 1][i], 6));
  });
  it('T7 hover has positive margin at SL and less in hot & high', () => {
    const sl = hoverPerformance({ product: t7, payload: null, altitudeM: 0, isaDeltaK: 0 }).value;
    const hh = hoverPerformance({ product: t7, payload: null, altitudeM: 2000, isaDeltaK: 25 }).value;
    expect(sl.powerMargin).toBeGreaterThan(0);
    expect(sl.thrustToWeight).toBeGreaterThan(1.2);
    expect(hh.powerMargin).toBeLessThan(sl.powerMargin);
    expect(sl.rotorThrustN.reduce((a, b) => a + b, 0)).toBeGreaterThan(sl.weightN * 0.999);
  });
  it('hover wind: tilt = atan(D/W), limit is monotonic', () => {
    const inp = { product: t7, payload: null, altitudeM: 0, isaDeltaK: 0 };
    const c = hoverWindCheck(inp, 10, 90);
    const W = hoverPerformance(inp).value.weightN;
    expect(c.tiltDeg).toBeCloseTo((Math.atan(c.dragN / W) * 180) / Math.PI, 6);
    const lim = hoverWindLimit(inp, 10, 90).value;
    expect(lim.maxWindMs).toBeGreaterThan(5);
    expect(hoverWindCheck(inp, lim.maxWindMs + 1, 90).limitedBy).not.toBe('ok');
    expect(lim.worstDirectionMaxWindMs).toBeLessThanOrEqual(lim.maxWindMs + 1e-9);
  });
});

describe('cruise / mission', () => {
  const inp = { product: t7, payload: payloadById(t7, 'cargo'), altitudeM: 0, isaDeltaK: 0, motors: 2 as const };
  it('speed ordering Vs < Vmp < Vbr < Vmax', () => {
    const c = cruisePerformance(inp).value;
    expect(c.stallSpeedMs).toBeLessThan(c.minPowerSpeedMs);
    expect(c.minPowerSpeedMs).toBeLessThanOrEqual(c.bestRangeSpeedMs);
    expect(c.bestRangeSpeedMs).toBeLessThan(c.maxSpeedMs);
  });
  it('4 motors give higher max speed but use more power at best-range', () => {
    const c2 = cruisePerformance(inp).value;
    const c4 = cruisePerformance({ ...inp, motors: 4 }).value;
    expect(c4.maxSpeedMs).toBeGreaterThan(c2.maxSpeedMs);
    expect(c4.bestRangeElectricW).toBeGreaterThan(c2.bestRangeElectricW);
  });
  it('mission energy closes and headwind reduces range', () => {
    const base = { product: t7, payload: inp.payload, mission: S.mission, altitudeM: 0, isaDeltaK: 0, motors: 2 as const, batterySoc: 1, windSpeedMs: 0, windFromDeg: 0 };
    const m0 = missionPerformance(base).value;
    const mw = missionPerformance({ ...base, windSpeedMs: 10 }).value;
    const sum = m0.hoverEnergyWh + m0.transitionEnergyWh + m0.cruiseEnergyWh + m0.reserveWh;
    expect(sum).toBeCloseTo(m0.energyAvailableWh, 6);
    expect(mw.rangeM).toBeLessThan(m0.rangeM);
    expect(m0.enduranceS).toBeGreaterThan(20 * 60);
  });
});

describe('wind models', () => {
  it('log profile equals reference at 10 m and grows with height', () => {
    expect(windAtHeight(10, 10)).toBeCloseTo(10, 9);
    expect(windAtHeight(10, 100)).toBeGreaterThan(10);
    expect(windAtHeight(10, 100, false)).toBe(10);
  });
  it('1-cosine gust peaks at mid-duration', () => {
    expect(oneMinusCosineGust(6, 8, 5, 2)).toBeCloseTo(8, 9);
    expect(oneMinusCosineGust(4.9, 8, 5, 2)).toBe(0);
  });
  it('Dryden is reproducible with the same seed and has the right σ', () => {
    const prm = drydenParams('moderate', 100).value;
    const a = new DrydenTurbulence(prm, 7), b = new DrydenTurbulence(prm, 7), c = new DrydenTurbulence(prm, 8);
    const sa: number[] = [], sb: number[] = [], sc: number[] = [];
    for (let i = 0; i < 20000; i++) { sa.push(a.step(0.05, 20)[0]); sb.push(b.step(0.05, 20)[0]); sc.push(c.step(0.05, 20)[0]); }
    expect(sa).toEqual(sb);
    expect(sa).not.toEqual(sc);
    const std = Math.sqrt(sa.reduce((s, x) => s + x * x, 0) / sa.length);
    expect(std).toBeGreaterThan(prm.sigmaMs[0] * 0.7);
    expect(std).toBeLessThan(prm.sigmaMs[0] * 1.3);
  });
});

describe('time-domain simulation', () => {
  const base: SimInput = {
    product: t7, payload: null,
    env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 0, windFromDeg: 90, turbulence: 'none', gustAmplitudeMs: 0, logProfile: true },
    mission: S.mission, mode: 'hover', cruiseMotors: 2, durationS: 15, seed: 1, gustStartS: 5, gustDurationS: 2,
  };
  it('calm hover holds position', () => {
    const tr = simulate(base);
    expect(tr.summary.maxPosErrorM).toBeLessThan(0.2);
    expect(tr.summary.positionLost).toBe(false);
  });
  it('is deterministic for a seed', () => {
    const inp = { ...base, env: { ...base.env, windSpeedMs: 8, turbulence: 'moderate' as const } };
    expect(simulate(inp).pos).toEqual(simulate(inp).pos);
  });
  it('wind below limit is held, wind above limit loses position', () => {
    const lim = hoverWindLimit({ product: t7, payload: null, altitudeM: 0, isaDeltaK: 0, controlReserveFraction: S.mission.controlReserveFraction }, 0, 90).value.maxWindMs;
    const ok = simulate({ ...base, env: { ...base.env, windSpeedMs: lim * 0.6, logProfile: false }, durationS: 25 });
    const bad = simulate({ ...base, env: { ...base.env, windSpeedMs: lim * 1.5, logProfile: false }, durationS: 25 });
    expect(ok.summary.positionLost).toBe(false);
    expect(bad.summary.positionLost).toBe(true);
  });
  it('transition reaches wing-borne flight without losing altitude', () => {
    const tr = simulate({ ...base, mode: 'transition', durationS: 30, cruiseMotors: 4 });
    expect(tr.foldDeg[tr.foldDeg.length - 1]).toBeCloseTo(0, 6);
    expect(tr.airspeedMs[tr.airspeedMs.length - 1]).toBeGreaterThan(15);
    expect(tr.summary.maxAltErrorM).toBeLessThan(5);
  });
  it('cruise in severe turbulence stays airborne', () => {
    const tr = simulate({ ...base, mode: 'cruise', env: { ...base.env, turbulence: 'severe', altitudeM: 300 }, durationS: 20 });
    expect(tr.summary.crashed).toBe(false);
  });
});
