/// <reference lib="webworker" />
import { sweepEnvelope, type EnvelopeInput } from '../physics/envelope';
import { simulate, type SimInput } from '../physics/sim';

type Req = { id: number; kind: 'sim'; input: SimInput } | { id: number; kind: 'envelope'; input: EnvelopeInput[] };

self.onmessage = (e: MessageEvent<Req>) => {
  const { id } = e.data;
  try {
    if (e.data.kind === 'sim') {
      self.postMessage({ id, result: simulate(e.data.input) });
    } else {
      self.postMessage({ id, result: e.data.input.map((inp) => sweepEnvelope(inp)) });
    }
  } catch (err) {
    self.postMessage({ id, error: err instanceof Error ? err.message : String(err) });
  }
};
