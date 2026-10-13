import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { validateSpecs, type SpecsFile } from '../data/schema';
import { sweepEnvelope } from '../physics/envelope';
import { useApp } from '../store/app';
import { decodeState, encodeState } from './urlState';

const specs = validateSpecs(JSON.parse(readFileSync(resolve(__dirname, '../../public/specs.json'), 'utf8'))).specs as SpecsFile;

describe('shareable URL state', () => {
  it('encodes and decodes product, payload and scenario', () => {
    useApp.setState({ specs, productId: 'T10-M', payloadId: 'sigint', scenarioId: 'hot_high', env: { ...specs.scenarios[2].env }, flightMode: 'cruise', cruiseMotors: 4, batterySoc: 0.7 });
    const hash = encodeState(useApp.getState());
    useApp.setState({ productId: 'T5-C', payloadId: 'none', scenarioId: null, flightMode: 'hover', cruiseMotors: 2, batterySoc: 1 });
    const patch = decodeState('#' + hash, useApp.getState());
    expect(patch.productId).toBe('T10-M');
    expect(patch.payloadId).toBe('sigint');
    expect(patch.scenarioId).toBe('hot_high');
    expect(patch.env?.altitudeM).toBe(2000);
    expect(patch.env?.isaDeltaK).toBe(25);
    expect(patch.flightMode).toBe('cruise');
    expect(patch.cruiseMotors).toBe(4);
    expect(patch.batterySoc).toBeCloseTo(0.7);
  });
  it('ignores unknown products', () => {
    expect(decodeState('#p=NOPE', useApp.getState()).productId).toBeUndefined();
  });
});

describe('envelope sweep', () => {
  it('is monotonic in wind; tilt-limited wind limit grows as density falls', () => {
    const r = sweepEnvelope({
      product: specs.products[2], payload: null, axis: 'altitude', windsMs: Array.from({ length: 31 }, (_, i) => i),
      rows: [0, 2000, 4000], altitudeM: 0, isaDeltaK: 20, windFromDeg: 90, controlReserveFraction: 0.15,
    }).value;
    for (const line of r.cells) for (let j = 1; j < line.length; j++) expect(line[j]).toBeGreaterThanOrEqual(line[j - 1]);
    expect(r.cells[0][0]).toBe(0);
    // tan φ = D/W with D ∝ ρV²: the same tilt allows a higher true wind speed in thin air
    expect(r.marginalLimit[2]).toBeGreaterThanOrEqual(r.marginalLimit[0]);
  });
});
