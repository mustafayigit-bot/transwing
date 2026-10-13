/**
 * Import of external results (JSBSim, PX4 SITL, CFD post-processing) as CSV:
 *   time, x, y, z, roll, pitch, yaw, thrust_1..n   (+ optional power_w, fold_deg)
 * Positions in metres, angles in degrees (radians auto-detected), thrust in newtons.
 * Frame: x forward/north, y right/east, z up (ENU-like). With `nedFrame` z is down (PX4/NED).
 */
import type { SimTrace } from '../physics/sim';

export interface CsvImportOptions {
  nedFrame: boolean;
  label: string;
  anglesInRadians?: boolean | 'auto';
}

export function parseCsvTrace(text: string, opts: CsvImportOptions): SimTrace {
  const lines = text.split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
  if (lines.length < 2) throw new Error('CSV en az bir başlık ve bir veri satırı içermeli.');
  const sep = lines[0].includes(';') && !lines[0].includes(',') ? ';' : lines[0].includes('\t') ? '\t' : ',';
  const head = lines[0].split(sep).map((h) => h.trim().toLowerCase().replace(/^"|"$/g, ''));
  const idx = (names: string[]) => head.findIndex((h) => names.includes(h));
  const col = {
    t: idx(['time', 't', 'time_s', 'timestamp']),
    x: idx(['x', 'x_m', 'north', 'n']),
    y: idx(['y', 'y_m', 'east', 'e']),
    z: idx(['z', 'z_m', 'up', 'alt', 'down', 'd']),
    roll: idx(['roll', 'phi', 'roll_deg']),
    pitch: idx(['pitch', 'theta', 'pitch_deg']),
    yaw: idx(['yaw', 'psi', 'yaw_deg', 'heading']),
    power: idx(['power_w', 'power']),
    fold: idx(['fold_deg', 'fold']),
  };
  const missing = (['t', 'x', 'y', 'z'] as const).filter((k) => col[k] < 0);
  if (missing.length) throw new Error(`Eksik sütun(lar): ${missing.join(', ')}. Beklenen: time,x,y,z,roll,pitch,yaw,thrust_1..n`);
  const thrustCols = head
    .map((h, i) => ({ h, i, m: /^thrust_?(\d+)$/.exec(h) }))
    .filter((c) => c.m)
    .sort((a, b) => Number(a.m![1]) - Number(b.m![1]))
    .map((c) => c.i);

  const rows = lines.slice(1).map((l) => l.split(sep).map((v) => Number(v.trim())));
  const valid = rows.filter((r) => Number.isFinite(r[col.t]));
  if (!valid.length) throw new Error('Geçerli veri satırı bulunamadı.');
  const angleCols = [col.roll, col.pitch, col.yaw].filter((c) => c >= 0);
  let rad = opts.anglesInRadians === true;
  if (opts.anglesInRadians === undefined || opts.anglesInRadians === 'auto') {
    const maxAbs = Math.max(0, ...valid.flatMap((r) => angleCols.map((c) => Math.abs(r[c] || 0))));
    rad = maxAbs > 0 && maxAbs <= 2 * Math.PI + 0.01 && head.every((h) => !h.endsWith('_deg'));
  }
  const toDeg = (v: number) => (Number.isFinite(v) ? (rad ? (v * 180) / Math.PI : v) : 0);
  const t0 = valid[0][col.t];
  const trace: SimTrace = {
    source: 'external',
    label: opts.label,
    mode: 'hover',
    t: [], pos: [], att: [], thrustN: [], saturated: [], powerW: [], windMs: [], foldDeg: [], airspeedMs: [],
    refPos: [0, 0, 0],
    summary: { maxPosErrorM: 0, rmsPosErrorM: 0, maxAltErrorM: 0, maxTiltDeg: 0, saturationEvents: 0, saturationTimeS: 0, energyWh: 0, peakPowerW: 0, meanPowerW: 0, positionLost: false, crashed: false },
    assumptions: [
      `Harici veri (${opts.label}). Eksen: x ileri, y sağ, z ${opts.nedFrame ? 'aşağı (NED)' : 'yukarı'}; açılar ${rad ? 'radyan → derece' : 'derece'}.`,
      'İtki sütunları yoksa itki 0 kabul edilir; doyma bilgisi harici veride yoktur.',
    ],
  };
  for (const r of valid) {
    const zUp = opts.nedFrame ? -r[col.z] : r[col.z];
    const yRight = r[col.y];
    trace.t.push(r[col.t] - t0);
    trace.pos.push([r[col.x], zUp, yRight]); // scene frame: x fwd, y up, z right
    const yaw = col.yaw >= 0 ? toDeg(r[col.yaw]) : 0;
    trace.att.push([col.roll >= 0 ? toDeg(r[col.roll]) : 0, col.pitch >= 0 ? toDeg(r[col.pitch]) : 0, yaw]);
    const th = thrustCols.map((c) => (Number.isFinite(r[c]) ? r[c] : 0));
    trace.thrustN.push(th);
    trace.saturated.push(th.map(() => false));
    trace.powerW.push(col.power >= 0 && Number.isFinite(r[col.power]) ? r[col.power] : NaN);
    trace.windMs.push([0, 0, 0]);
    trace.foldDeg.push(col.fold >= 0 && Number.isFinite(r[col.fold]) ? r[col.fold] : NaN);
    trace.airspeedMs.push(NaN);
  }
  // reference = first sample; error metrics relative to it
  const ref = trace.pos[0];
  trace.refPos = [...ref] as [number, number, number];
  let maxE = 0, sum = 0, maxAlt = 0, maxTilt = 0;
  trace.pos.forEach((p, i) => {
    const e = Math.hypot(p[0] - ref[0], p[1] - ref[1], p[2] - ref[2]);
    maxE = Math.max(maxE, e);
    sum += e * e;
    maxAlt = Math.max(maxAlt, Math.abs(p[1] - ref[1]));
    maxTilt = Math.max(maxTilt, Math.hypot(trace.att[i][0], trace.att[i][1]));
  });
  trace.summary.maxPosErrorM = maxE;
  trace.summary.rmsPosErrorM = Math.sqrt(sum / trace.pos.length);
  trace.summary.maxAltErrorM = maxAlt;
  trace.summary.maxTiltDeg = maxTilt;
  const pw = trace.powerW.filter(Number.isFinite);
  if (pw.length) {
    trace.summary.peakPowerW = Math.max(...pw);
    trace.summary.meanPowerW = pw.reduce((a, b) => a + b, 0) / pw.length;
  }
  return trace;
}
