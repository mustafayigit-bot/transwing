import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { validateSpecs, type SpecsFile } from '../data/schema';
import { simulate } from '../physics/sim';
import { parseCsvTrace } from './csvImport';
import { traceToCsv } from './exporters';

const specs = validateSpecs(JSON.parse(readFileSync(resolve(__dirname, '../../public/specs.json'), 'utf8'))).specs as SpecsFile;

describe('CSV import / export', () => {
  it('round-trips an internal trace through the external CSV format', () => {
    const tr = simulate({
      product: specs.products[2], payload: null,
      env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 8, windFromDeg: 60, turbulence: 'light', gustAmplitudeMs: 0, logProfile: true },
      mission: specs.mission, mode: 'hover', cruiseMotors: 2, durationS: 4, seed: 3, gustStartS: 1, gustDurationS: 1,
    });
    const back = parseCsvTrace(traceToCsv(tr), { nedFrame: false, label: 'rt' });
    expect(back.t.length).toBe(tr.t.length);
    const k = Math.floor(tr.t.length / 2);
    tr.pos[k].forEach((v, i) => expect(back.pos[k][i]).toBeCloseTo(v, 3));
    tr.att[k].forEach((v, i) => expect(back.att[k][i]).toBeCloseTo(v, 2));
    expect(back.thrustN[k]).toHaveLength(4);
  });
  it('handles NED frames and radian angles', () => {
    const csv = 'time,x,y,z,roll,pitch,yaw,thrust_1,thrust_2\n0,0,0,-10,0,0,0,100,100\n1,1,2,-12,0.1,0.2,0.3,110,90\n';
    const tr = parseCsvTrace(csv, { nedFrame: true, label: 'px4' });
    expect(tr.pos[1]).toEqual([1, 12, 2]);
    expect(tr.att[1][0]).toBeCloseTo(0.1 * 180 / Math.PI, 6);
    expect(tr.summary.maxAltErrorM).toBeCloseTo(2, 9);
  });
  it('rejects missing columns', () => {
    expect(() => parseCsvTrace('time,a,b\n0,1,2\n', { nedFrame: false, label: 'x' })).toThrow(/Eksik/);
  });
});
