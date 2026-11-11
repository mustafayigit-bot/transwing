import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { validateSpecs, type Product, type SpecsFile } from '../data/schema';
import { simulateMission, type MissionConfig } from '../physics/fdm/mission';
import { encodeDataFlash, messageLength } from './dataflash';
import { encodeTlog, x25 } from './mavlink';
import { synthesizeTelemetry } from './sensors';
import { aggregateMonteCarlo, evaluateAlarms, phaseStats, rangeOf } from './stats';

const specs = validateSpecs(JSON.parse(readFileSync(resolve(__dirname, '../../public/specs.json'), 'utf8'))).specs as SpecsFile;
const tw4 = specs.products.find((p) => p.id === 'TW4') as Product;
const cfg: MissionConfig = {
  hoverHeightM: 20, cruiseAltitudeM: 100, cruiseSpeedMs: 23.7, takeoffHoverS: 20, landingHoverS: 30, waypointsM: [[900, 0]], cruiseMotors: 4,
  env: { altitudeM: 0, isaDeltaK: 0, windSpeedMs: 4, windFromDeg: 30, turbulence: 'light', gustAmplitudeMs: 0, logProfile: true },
  seed: 5, gustStartS: 5, gustDurationS: 2, initialSoc: 1, cdaFront: 0.01877,
};
const trace = simulateMission(tw4, tw4.payloadOptions[0], cfg);
const tel = synthesizeTelemetry(trace, tw4);

describe('telemetry synthesis', () => {
  it('is deterministic and covers the ArduPilot message set', () => {
    const again = synthesizeTelemetry(trace, tw4);
    expect(again.ch.accZ).toEqual(tel.ch.accZ);
    const names = tel.streams.map((s) => s.name);
    for (const n of ['ATT', 'IMU', 'VIBE', 'GPS', 'BARO', 'ARSP', 'BAT', 'ESC', 'RCOU', 'CTUN', 'QTUN', 'MODE', 'MSG', 'HNG', 'LOAD']) expect(names).toContain(n);
  });
  it('IMU reads about −1 g on the z axis in hover and vibration is higher in hover than in cruise', () => {
    const idx = (ph: number) => trace.ch.phase.map((p, i) => (p === ph ? i : -1)).filter((i) => i >= 0);
    const hover = idx(1), cruise = idx(4).length ? idx(4) : idx(3);
    const mean = (k: string, ii: number[]) => ii.reduce((s, i) => s + tel.ch[k][i], 0) / ii.length;
    expect(mean('accZ', hover)).toBeGreaterThan(-10.6);
    expect(mean('accZ', hover)).toBeLessThan(-9.0);
    expect(mean('vibeZ', hover)).toBeGreaterThan(mean('vibeX', cruise) * 0 + 1);
    expect(mean('vibeX', hover)).toBeGreaterThan(mean('vibeX', cruise));
  });
  it('sensor errors stay within plausible bounds', () => {
    const err = tel.ch.gpsAlt.map((v, i) => Math.abs(v - trace.ch.alt[i]));
    expect(Math.max(...err)).toBeLessThan(8);
    expect(Math.max(...tel.ch.gpsErrH)).toBeLessThan(5);
  });
});

describe('DataFlash writer', () => {
  const bin = encodeDataFlash(tel);
  it('starts with a self-describing FMT record', () => {
    expect([bin[0], bin[1], bin[2]]).toEqual([0xa3, 0x95, 128]);
    expect(bin[3]).toBe(128);
    expect(bin[4]).toBe(89);
    expect(String.fromCharCode(bin[5], bin[6], bin[7])).toBe('FMT');
  });
  it('has the exact size implied by the formats', () => {
    const streams = tel.streams.filter((s) => s.rows.length);
    const expected = 89 * (streams.length + 1) + streams.reduce((s, x) => s + messageLength(x.format) * x.rows.length, 0);
    expect(bin.length).toBe(expected);
    expect(messageLength('QBfffI')).toBe(3 + 8 + 1 + 12 + 4);
  });
});

describe('MAVLink tlog writer', () => {
  it('computes the reference X.25 checksum', () => {
    expect(x25(new TextEncoder().encode('123456789'), 0, 9)).toBe(0x6f91);
  });
  it('frames messages with valid checksums', () => {
    const log = encodeTlog(trace, tel);
    const extras: Record<number, number> = { 0: 50, 1: 124, 24: 24, 30: 39, 33: 104, 36: 222, 62: 183, 74: 20, 147: 154, 168: 1, 241: 90, 253: 83 };
    let off = 0, count = 0;
    while (off < log.length) {
      expect(log[off + 8]).toBe(0xfe);
      const len = log[off + 9], id = log[off + 13];
      const tmp = new Uint8Array(5 + len + 1);
      tmp.set(log.subarray(off + 9, off + 14 + len));
      tmp[5 + len] = extras[id];
      const crc = x25(tmp, 0, tmp.length);
      expect(log[off + 14 + len] | (log[off + 15 + len] << 8)).toBe(crc);
      off += 16 + len;
      count++;
    }
    expect(count).toBeGreaterThan(500);
  });
});

describe('statistics', () => {
  it('ranges and per-phase statistics', () => {
    const r = rangeOf([1, 2, 3, 4, 5]);
    expect(r.min).toBe(1); expect(r.max).toBe(5); expect(r.mean).toBe(3);
    const st = phaseStats(trace.ch.phase, trace.ch.t, trace.ch.power);
    const hover = st.find((p) => p.phase === 1)!, all = st.find((p) => p.phase === -1)!;
    expect(hover.range.mean).toBeGreaterThan(5000);
    expect(all.range.max).toBeGreaterThanOrEqual(hover.range.max);
  });
  it('alarm table flags the hover C-rate and keeps cell voltage green', () => {
    const a = evaluateAlarms(trace, tel.ch, tw4);
    expect(a.find((x) => x.id === 'cellV')!.status).toBe('green');
    expect(a.find((x) => x.id === 'cRate')!.worst).toBeGreaterThan(2);
    expect(a.every((x) => Number.isFinite(x.worst))).toBe(true);
  });
  it('aggregates Monte-Carlo runs', () => {
    const runs = [1, 2, 3].map((seed) => ({ seed, summary: { ...trace.summary, energyWh: 100 * seed }, maxVibe: seed }));
    const res = aggregateMonteCarlo(runs, [0, 1], { alt: [[1, 2], [3, 4], [5, NaN]] });
    expect(res.successRate).toBe(1);
    expect(res.metrics.find((m) => m.id === 'energyWh')!.range.max).toBe(300);
    expect(res.bands.ch.alt.p50[0]).toBe(3);
  });
});
