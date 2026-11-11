/// <reference lib="webworker" />
import type { PayloadOption, Product } from '../data/schema';
import { sweepEnvelope, type EnvelopeInput } from '../physics/envelope';
import { simulateMission, type MissionConfig } from '../physics/fdm/mission';
import { analyze } from '../physics/fdm/trim';
import { simulate, type SimInput } from '../physics/sim';
import { synthesizeTelemetry } from '../telemetry/sensors';
import { aggregateMonteCarlo, MC_BAND_KEYS, type MonteCarloRun } from '../telemetry/stats';

export interface MissionRequest {
  product: Product;
  payload: PayloadOption | null;
  config: MissionConfig;
}
export interface AnalysisRequest {
  product: Product;
  payload: PayloadOption | null;
  altitudeM: number;
  isaDeltaK: number;
  cruiseSpeedMs?: number;
}

type Req =
  | { id: number; kind: 'sim'; input: SimInput }
  | { id: number; kind: 'envelope'; input: EnvelopeInput[] }
  | { id: number; kind: 'mission'; input: MissionRequest }
  | { id: number; kind: 'analysis'; input: AnalysisRequest }
  | { id: number; kind: 'montecarlo'; input: MissionRequest & { runs: number } };

self.onmessage = (e: MessageEvent<Req>) => {
  const { id } = e.data;
  try {
    switch (e.data.kind) {
      case 'sim':
        self.postMessage({ id, result: simulate(e.data.input) });
        break;
      case 'envelope':
        self.postMessage({ id, result: e.data.input.map((inp) => sweepEnvelope(inp)) });
        break;
      case 'mission': {
        const { product, payload, config } = e.data.input;
        const trace = simulateMission(product, payload, config, (frac, phase) => self.postMessage({ id, progress: frac, phase }));
        self.postMessage({ id, result: trace });
        break;
      }
      case 'montecarlo': {
        const { product, payload, config, runs } = e.data.input;
        const done: MonteCarloRun[] = [];
        const traces: { t: number[]; ch: Record<string, number[]> }[] = [];
        let cda = config.cdaFront;
        for (let k = 0; k < runs; k++) {
          const seed = config.seed + k;
          const tr = simulateMission(product, payload, { ...config, seed, cdaFront: cda, outputHz: 2 }, (frac, phase) => self.postMessage({ id, progress: (k + frac) / runs, phase }));
          cda = tr.cdaFront;
          const tel = synthesizeTelemetry(tr, product, { seed });
          let maxVibe = 0;
          for (let i = 0; i < tr.ch.t.length; i++) maxVibe = Math.max(maxVibe, tel.ch.vibeX[i], tel.ch.vibeY[i], tel.ch.vibeZ[i]);
          done.push({ seed, summary: tr.summary, maxVibe });
          const keep: Record<string, number[]> = {};
          for (const key of MC_BAND_KEYS) keep[key] = tr.ch[key];
          traces.push({ t: tr.ch.t, ch: keep });
          self.postMessage({ id, progress: (k + 1) / runs, phase: 0 });
        }
        // common time grid up to the longest run; shorter runs contribute NaN beyond their end
        const tMax = Math.max(...traces.map((t) => t.t[t.t.length - 1] ?? 0));
        const N = 300;
        const grid = Array.from({ length: N }, (_, j) => (tMax * j) / (N - 1));
        const series: Record<string, number[][]> = {};
        for (const key of MC_BAND_KEYS) {
          series[key] = traces.map((tr) => {
            const dt = tr.t.length > 1 ? tr.t[1] - tr.t[0] : 1;
            return grid.map((g) => {
              const i = Math.round(g / dt);
              return i < tr.t.length ? tr.ch[key][i] : NaN;
            });
          });
        }
        self.postMessage({ id, result: aggregateMonteCarlo(done, grid, series) });
        break;
      }
      case 'analysis': {
        const a = e.data.input;
        self.postMessage({ id, result: analyze(a.product, a.payload, a.altitudeM, a.isaDeltaK, a.cruiseSpeedMs) });
        break;
      }
    }
  } catch (err) {
    self.postMessage({ id, error: err instanceof Error ? err.message : String(err) });
  }
};
