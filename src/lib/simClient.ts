/** Runs simulations and envelope sweeps in a Web Worker so the UI never blocks. */
import type { EnvelopeInput, EnvelopeResult } from '../physics/envelope';
import type { SimInput, SimTrace } from '../physics/sim';
import type { Result } from '../physics/types';
import { payloadById } from '../physics/massProps';
import { useApp } from '../store/app';

let worker: Worker | null = null;
let nextId = 1;
const pending = new Map<number, { resolve: (t: unknown) => void; reject: (e: Error) => void }>();

function getWorker(): Worker {
  if (!worker) {
    worker = new Worker(new URL('../workers/sim.worker.ts', import.meta.url), { type: 'module' });
    worker.onmessage = (e: MessageEvent<{ id: number; result?: unknown; error?: string }>) => {
      const p = pending.get(e.data.id);
      if (!p) return;
      pending.delete(e.data.id);
      if (e.data.error) p.reject(new Error(e.data.error));
      else p.resolve(e.data.result);
    };
  }
  return worker;
}

function call<T>(kind: 'sim' | 'envelope', input: unknown): Promise<T> {
  const id = nextId++;
  return new Promise<T>((resolve, reject) => {
    pending.set(id, { resolve: resolve as (t: unknown) => void, reject });
    getWorker().postMessage({ id, kind, input });
  });
}

export const runSimulation = (input: SimInput) => call<SimTrace>('sim', input);
export const runEnvelope = (inputs: EnvelopeInput[]) => call<Result<EnvelopeResult>[]>('envelope', inputs);

/** Build the simulation input from the current app state for a product/payload. */
export function currentSimInput(productId?: string, payloadId?: string): SimInput | null {
  const s = useApp.getState();
  const product = s.specs?.products.find((p) => p.id === (productId ?? s.productId));
  if (!product || !s.specs) return null;
  return {
    product,
    payload: payloadById(product, payloadId ?? s.payloadId),
    env: s.env,
    mission: s.specs.mission,
    mode: s.flightMode,
    cruiseMotors: s.cruiseMotors,
    durationS: s.sim.durationS,
    seed: s.sim.seed,
    gustStartS: s.sim.gustStartS,
    gustDurationS: s.sim.gustDurationS,
  };
}

export async function runCurrentSimulation(): Promise<SimTrace | null> {
  const input = currentSimInput();
  if (!input) return null;
  useApp.setState({ simRunning: true, simError: null });
  try {
    const trace = await runSimulation(input);
    useApp.setState({ trace, simRunning: false, replayT: 0, replayPlaying: true, replaySource: 'internal' });
    return trace;
  } catch (e) {
    useApp.setState({ simRunning: false, simError: e instanceof Error ? e.message : String(e) });
    return null;
  }
}
