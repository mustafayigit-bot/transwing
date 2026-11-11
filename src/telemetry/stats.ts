/** Telemetry statistics: per-phase ranges, alarm evaluation and Monte-Carlo aggregation. */
import type { Product } from '../data/schema';
import { PHASES, type MissionSummary, type MissionTrace } from '../physics/fdm/mission';

export interface Range {
  min: number;
  mean: number;
  max: number;
  p05: number;
  p95: number;
}

export function rangeOf(values: number[]): Range {
  const v = values.filter(Number.isFinite);
  if (!v.length) return { min: NaN, mean: NaN, max: NaN, p05: NaN, p95: NaN };
  const s = [...v].sort((a, b) => a - b);
  const q = (p: number) => s[Math.min(s.length - 1, Math.max(0, Math.round(p * (s.length - 1))))];
  return { min: s[0], mean: v.reduce((a, b) => a + b, 0) / v.length, max: s[s.length - 1], p05: q(0.05), p95: q(0.95) };
}

export interface PhaseStat {
  phase: number;
  name: string;
  durationS: number;
  range: Range;
}

/** Statistics of one channel for every flight phase (plus the whole flight as phase −1). */
export function phaseStats(phase: number[], t: number[], values: number[]): PhaseStat[] {
  const out: PhaseStat[] = [];
  const dt = t.length > 1 ? t[1] - t[0] : 0;
  for (let p = 0; p < PHASES.length - 1; p++) {
    const sel: number[] = [];
    for (let i = 0; i < phase.length; i++) if (phase[i] === p) sel.push(values[i]);
    if (sel.length) out.push({ phase: p, name: PHASES[p], durationS: sel.length * dt, range: rangeOf(sel) });
  }
  out.push({ phase: -1, name: 'Tüm görev', durationS: t.length * dt, range: rangeOf(values) });
  return out;
}

export interface AlarmRow {
  id: string;
  label: string;
  unit: string;
  /** worst observed value */
  worst: number;
  warn: number;
  critical: number;
  /** true when larger values are worse */
  highIsBad: boolean;
  timeBeyondWarnS: number;
  status: 'green' | 'yellow' | 'red';
  where: string;
}

/** Default vibration thresholds follow ArduPilot guidance (VIBE < 30 m/s² good, > 60 m/s² bad). */
export const VIBE_WARN = 30, VIBE_CRIT = 60;

export function evaluateAlarms(trace: MissionTrace, sensorCh: Record<string, number[]>, product: Product): AlarmRow[] {
  const c = trace.ch;
  const f = product.fdm!;
  const dt = c.t.length > 1 ? c.t[1] - c.t[0] : 0;
  const nR = trace.nRotors;
  const maxOver = (keys: string[], src: Record<string, number[]>) => src[keys[0]].map((_, i) => Math.max(...keys.map((k) => Math.abs(src[k][i]))));
  const row = (id: string, label: string, unit: string, v: number[], warn: number, critical: number, highIsBad: boolean): AlarmRow => {
    let worst = highIsBad ? -Infinity : Infinity, wi = 0, beyond = 0;
    for (let i = 0; i < v.length; i++) {
      if (!Number.isFinite(v[i])) continue;
      if (highIsBad ? v[i] > worst : v[i] < worst) { worst = v[i]; wi = i; }
      if (highIsBad ? v[i] > warn : v[i] < warn) beyond += dt;
    }
    const bad = (lim: number) => (highIsBad ? worst > lim : worst < lim);
    return { id, label, unit, worst, warn, critical, highIsBad, timeBeyondWarnS: beyond, status: bad(critical) ? 'red' : bad(warn) ? 'yellow' : 'green', where: `${PHASES[c.phase[wi]] ?? ''} · t = ${c.t[wi]?.toFixed(0)} s` };
  };
  const rot = (k: string) => Array.from({ length: nR }, (_, i) => `${k}${i + 1}`);
  const fwOnly = (v: number[], fill: number) => v.map((x, i) => (c.phase[i] >= 3 && c.phase[i] <= 5 ? x : fill));
  return [
    row('cellV', 'Hücre gerilimi (BATT_LOW / CRT_VOLT)', 'V', c.cellV, f.battery.lowCellV, f.battery.criticalCellV, false),
    row('soc', 'Kalan kapasite (SoC)', '%', c.soc, f.battery.reserveSoc * 100 + 5, f.battery.reserveSoc * 100, false),
    row('cRate', 'Batarya akımı', 'C', c.cRate, f.battery.maxContinuousC, f.battery.peakC, true),
    row('cur', 'Motor faz akımı', 'A', maxOver(rot('cur'), c), 0.85 * f.motor.maxCurrentA, f.motor.maxCurrentA, true),
    row('tmot', 'Motor sıcaklığı', '°C', maxOver(rot('tmot'), c), 0.85 * f.motor.maxTempC, f.motor.maxTempC, true),
    row('tesc', 'ESC sıcaklığı', '°C', maxOver(rot('tesc'), c), 0.85 * f.esc.maxTempC, f.esc.maxTempC, true),
    row('cellT', 'Hücre sıcaklığı', '°C', c.cellT, 50, 60, true),
    row('vibe', 'Titreşim (VIBE, en kötü eksen)', 'm/s²', maxOver(['vibeX', 'vibeY', 'vibeZ'], sensorCh), VIBE_WARN, VIBE_CRIT, true),
    row('hinge', 'Menteşe momenti', 'N·m', maxOver(['hingeL', 'hingeR'], c), 0.8 * f.hingeActuator.maxTorqueNm, f.hingeActuator.maxTorqueNm, true),
    row('ias', 'Sabit kanatta hava hızı (Q_ASSIST / stall)', 'm/s', fwOnly(c.ias, 99), f.transition.fbwMinSpeedMs - 0.5, f.transition.assistSpeedMs, false),
    row('stall', 'Stall olmuş kanat oranı (sabit kanat)', '–', fwOnly(c.stall, 0), 0.15, 0.3, true),
    row('bank', 'Yatış açısı', '°', c.roll.map(Math.abs), 35, 50, true),
    row('nz', 'Yük faktörü', 'g', c.nz.map(Math.abs), 1.8, 2.5, true),
    row('altErr', 'Seyirde irtifa hatası', 'm', c.alt.map((a, i) => (c.phase[i] === 4 ? Math.abs(a - c.altCmd[i]) : 0)), 10, 25, true),
  ];
}

export interface MonteCarloRun {
  seed: number;
  summary: MissionSummary;
  maxVibe: number;
}

export interface MonteCarloResult {
  runs: MonteCarloRun[];
  successRate: number;
  metrics: { id: string; label: string; unit: string; range: Range }[];
  /** percentile bands over mission time on a common grid */
  bands: { t: number[]; ch: Record<string, { p05: number[]; p50: number[]; p95: number[] }> };
  outcomes: Record<string, number>;
}

export const MC_METRICS: { id: keyof MissionSummary | 'maxVibe'; label: string; unit: string; scale?: number }[] = [
  { id: 'durationS', label: 'Görev süresi', unit: 'dk', scale: 1 / 60 },
  { id: 'energyWh', label: 'Enerji', unit: 'Wh' },
  { id: 'finalSoc', label: 'Son SoC', unit: '%', scale: 100 },
  { id: 'minCellV', label: 'Min. hücre gerilimi', unit: 'V' },
  { id: 'maxCRate', label: 'Maks. batarya akımı', unit: 'C' },
  { id: 'cruiseMeanPowerW', label: 'Seyir ort. gücü', unit: 'W' },
  { id: 'hoverMeanPowerW', label: 'Hover ort. gücü', unit: 'W' },
  { id: 'transitionAltLossM', label: 'İleri geçiş irtifa kaybı', unit: 'm' },
  { id: 'maxHingeMomentNm', label: 'Maks. menteşe momenti', unit: 'N·m' },
  { id: 'maxMotorTempC', label: 'Maks. motor sıcaklığı', unit: '°C' },
  { id: 'maxBankDeg', label: 'Maks. yatış', unit: '°' },
  { id: 'maxLoadFactor', label: 'Maks. yük faktörü', unit: 'g' },
  { id: 'saturationTimeS', label: 'Rotor doyma süresi', unit: 's' },
  { id: 'stallTimeS', label: 'Stall süresi', unit: 's' },
  { id: 'assistTimeS', label: 'Q_ASSIST altı süre', unit: 's' },
  { id: 'touchdownSinkMs', label: 'Teker koyma hızı', unit: 'm/s' },
  { id: 'maxVibe', label: 'Maks. titreşim', unit: 'm/s²' },
];

export const MC_BAND_KEYS = ['alt', 'ias', 'power', 'battV', 'soc', 'roll', 'pitch'] as const;

/** Aggregate finished runs (band data: per run, per key, values on the common time grid). */
export function aggregateMonteCarlo(runs: MonteCarloRun[], grid: number[], series: Record<string, number[][]>): MonteCarloResult {
  const ok = runs.filter((r) => r.summary.completed);
  const metrics = MC_METRICS.map((m) => ({
    id: m.id as string, label: m.label, unit: m.unit,
    range: rangeOf(runs.map((r) => (m.id === 'maxVibe' ? r.maxVibe : (r.summary[m.id] as number)) * (m.scale ?? 1))),
  }));
  const bands: MonteCarloResult['bands'] = { t: grid, ch: {} };
  for (const k of Object.keys(series)) {
    const p05: number[] = [], p50: number[] = [], p95: number[] = [];
    for (let j = 0; j < grid.length; j++) {
      const col = series[k].map((s) => s[j]).filter(Number.isFinite).sort((a, b) => a - b);
      const q = (p: number) => (col.length ? col[Math.round(p * (col.length - 1))] : NaN);
      p05.push(q(0.05)); p50.push(q(0.5)); p95.push(q(0.95));
    }
    bands.ch[k] = { p05, p50, p95 };
  }
  const outcomes: Record<string, number> = {};
  for (const r of runs) {
    const key = r.summary.completed ? 'Tamamlandı' : r.summary.outcome.replace(/\s*\(.*$/, '').replace(/ — .*$/, '');
    outcomes[key] = (outcomes[key] ?? 0) + 1;
  }
  return { runs, successRate: runs.length ? ok.length / runs.length : 0, metrics, bands, outcomes };
}
