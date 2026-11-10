/// <reference lib="webworker" />
import type { PayloadOption, Product } from '../data/schema';
import { sweepEnvelope, type EnvelopeInput } from '../physics/envelope';
import { simulateMission, type MissionConfig } from '../physics/fdm/mission';
import { analyze } from '../physics/fdm/trim';
import { simulate, type SimInput } from '../physics/sim';

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
  | { id: number; kind: 'analysis'; input: AnalysisRequest };

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
