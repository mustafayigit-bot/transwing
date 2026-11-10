import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { validateSpecs, type Product, type SpecsFile } from '../../data/schema';
import { simulateMission, type MissionConfig } from './mission';
import { buildModel, GeomState } from './model';
import { analyze, eigenvalues } from './trim';
import { solveVlm } from './vlm';

const specs = validateSpecs(JSON.parse(readFileSync(resolve(__dirname, '../../../public/specs.json'), 'utf8'))).specs as SpecsFile;
const tw4 = specs.products.find((p) => p.id === 'TW4') as Product;
const mtow = tw4.payloadOptions[0];

describe('vortex lattice', () => {
  it('matches lifting-line theory for a high aspect ratio wing', () => {
    const v = solveVlm({ spanM: 10, chordRootM: 1, chordTipM: 1 });
    const AR = 10;
    const helmbold = (2 * Math.PI * AR) / (2 + Math.sqrt(4 + AR * AR));
    expect(v.clAlphaPerRad).toBeGreaterThan(helmbold * 0.92);
    expect(v.clAlphaPerRad).toBeLessThan(helmbold * 1.05);
    expect(v.spanEfficiency).toBeGreaterThan(0.9);
    expect(v.spanEfficiency).toBeLessThan(1.05);
  });
});

describe('eigenvalues', () => {
  it('finds the roots of a companion matrix', () => {
    // (λ+1)(λ+2)(λ²+2λ+5): roots −1, −2, −1±2i
    const ev = eigenvalues([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [-10, -19, -13, -5]]);
    const re = ev.map((e) => e.re).sort((a, b) => a - b);
    expect(re[0]).toBeCloseTo(-2, 6);
    expect(Math.max(...ev.map((e) => Math.abs(e.im)))).toBeCloseTo(2, 6);
  });
});

describe('Transwing 4.0 model', () => {
  it('has 40 kg MTOW and thrust axes vertical in hover', () => {
    const m = buildModel(tw4, mtow);
    expect(m.massKg).toBeCloseTo(40, 3);
    const g = new GeomState(m);
    g.update(m.maxFoldRad);
    for (let k = 0; k < m.rotors.length; k++) expect(g.taxis[3 * k + 1]).toBeGreaterThan(0.99);
    g.update(0);
    for (let k = 0; k < m.rotors.length; k++) expect(g.taxis[3 * k]).toBeGreaterThan(0.999);
  });
  const rep = analyze(tw4, mtow, 0, 0);
  it('reproduces the design-team stall speed and maximum L/D', () => {
    expect(rep.stallSpeedMs).toBeGreaterThan(19);
    expect(rep.stallSpeedMs).toBeLessThan(21.5);
    expect(rep.calibration.maxLD).toBeCloseTo(18.07, 1);
    expect(rep.cruise4.converged).toBe(true);
  });
  it('is statically stable with ≈10 % static margin and a damped short period', () => {
    expect(rep.derivatives.staticMargin).toBeGreaterThan(0.06);
    expect(rep.derivatives.staticMargin).toBeLessThan(0.14);
    expect(rep.derivatives.Cnbeta).toBeGreaterThan(0);
    expect(rep.derivatives.Clp).toBeLessThan(0);
    const sp = rep.modes.find((x) => x.name === 'Kısa periyot')!;
    expect(sp.stable).toBe(true);
    expect(sp.damping).toBeGreaterThan(0.3);
  });
  it('hover trim balances weight and two-motor cruise needs less power', () => {
    expect(rep.hover.converged).toBe(true);
    expect(rep.hover.thrustN.reduce((a, b) => a + b, 0)).toBeGreaterThan(40 * 9.80665);
    expect(rep.cruise2!.elecPowerW).toBeLessThan(rep.cruise4.elecPowerW);
  });
});

describe('mission simulation', () => {
  const cfg: MissionConfig = {
    hoverHeightM: 20, cruiseAltitudeM: 120, cruiseSpeedMs: 23.7, takeoffHoverS: 20, landingHoverS: 30, waypointsM: [[1500, 0]], cruiseMotors: 4,
    env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 0, windFromDeg: 0, turbulence: 'none', gustAmplitudeMs: 0, logProfile: true },
    seed: 3, gustStartS: 5, gustDurationS: 2, initialSoc: 1, cdaFront: 0.01877,
  };
  const tr = simulateMission(tw4, mtow, cfg);
  it('flies take-off → transition → cruise → back-transition → landing', () => {
    expect(tr.summary.completed).toBe(true);
    expect(tr.phases.map((p) => p.phase)).toEqual(expect.arrayContaining([0, 1, 2, 3, 6, 7, 8]));
    expect(tr.summary.touchdownSinkMs).toBeLessThan(1);
    expect(tr.summary.transitionAltLossM).toBeLessThan(8);
    expect(Math.max(...tr.ch.alt)).toBeGreaterThan(100);
    expect(Math.min(...tr.ch.fold)).toBeCloseTo(0, 3);
  });
  it('energy bookkeeping is consistent', () => {
    const sum = tr.phases.reduce((s, p) => s + p.energyWh, 0);
    expect(sum).toBeCloseTo(tr.summary.energyWh, 0);
    expect(tr.summary.finalSoc).toBeLessThan(1);
    expect(tr.summary.finalSoc).toBeGreaterThan(0.8);
  });
  it('is reproducible with turbulence for a fixed seed', () => {
    const c2: MissionConfig = { ...cfg, waypointsM: [[600, 0]], env: { ...cfg.env, windSpeedMs: 5, windFromDeg: 40, turbulence: 'light' } };
    const a = simulateMission(tw4, mtow, c2), b = simulateMission(tw4, mtow, c2);
    expect(a.ch.alt).toEqual(b.ch.alt);
    expect(a.summary.energyWh).toBe(b.summary.energyWh);
  });
});
