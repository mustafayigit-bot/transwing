import { create } from 'zustand';
import type { Scenario, SpecsFile, ValidationIssue } from '../data/schema';
import type { SimTrace } from '../physics/sim';
import type { Environment, FlightMode } from '../physics/types';

export type Tab = 'config' | 'compare' | 'envelope' | 'matrix';
export type CameraPreset = 'front' | 'side' | 'top' | 'iso';

export interface SimSettings {
  durationS: number;
  seed: number;
  gustStartS: number;
  gustDurationS: number;
}

export interface AppState {
  specs: SpecsFile | null;
  issues: ValidationIssue[];
  loadError: string | null;

  tab: Tab;
  presentation: boolean;

  productId: string;
  payloadId: string;
  scenarioId: string | null;
  env: Environment;
  flightMode: FlightMode;
  cruiseMotors: 2 | 4;
  batterySoc: number;
  sim: SimSettings;

  foldDeg: number;
  transitionPlaying: boolean;
  transitionDurationS: number;

  explode: number;
  showConnectors: boolean;
  wireframe: boolean;
  sectionEnabled: boolean;
  sectionOffset: number;
  showCG: boolean;
  showForces: boolean;
  showWind: boolean;
  showHuman: boolean;
  cameraRequest: { preset: CameraPreset; n: number } | null;

  hiddenParts: string[];
  isolatedPart: string | null;
  selectedPart: string | null;
  hoveredPart: string | null;

  trace: SimTrace | null;
  externalTrace: SimTrace | null;
  replaySource: 'internal' | 'external';
  simRunning: boolean;
  simError: string | null;
  replayT: number;
  replayPlaying: boolean;
  replaySpeed: number;

  compareB: { productId: string; payloadId: string };
  envelopeAxis: 'altitude' | 'payload';
  envelopeOverlay: boolean;

  set: (p: Partial<AppState>) => void;
  setEnv: (p: Partial<Environment>) => void;
  setSim: (p: Partial<SimSettings>) => void;
  applyScenario: (s: Scenario) => void;
  selectProduct: (id: string) => void;
  togglePart: (id: string) => void;
  requestCamera: (preset: CameraPreset) => void;
}

export const DEFAULT_ENV: Environment = {
  altitudeM: 0,
  isaDeltaK: 0,
  windSpeedMs: 0,
  windFromDeg: 90,
  turbulence: 'none',
  gustAmplitudeMs: 0,
  logProfile: true,
};

export const useApp = create<AppState>((set, get) => ({
  specs: null,
  issues: [],
  loadError: null,
  tab: 'config',
  presentation: false,
  productId: '',
  payloadId: 'none',
  scenarioId: null,
  env: DEFAULT_ENV,
  flightMode: 'hover',
  cruiseMotors: 2,
  batterySoc: 1,
  sim: { durationS: 30, seed: 42, gustStartS: 5, gustDurationS: 2 },
  foldDeg: 0,
  transitionPlaying: false,
  transitionDurationS: 6,
  explode: 0,
  showConnectors: true,
  wireframe: false,
  sectionEnabled: false,
  sectionOffset: 0,
  showCG: true,
  showForces: true,
  showWind: true,
  showHuman: true,
  cameraRequest: null,
  hiddenParts: [],
  isolatedPart: null,
  selectedPart: null,
  hoveredPart: null,
  trace: null,
  externalTrace: null,
  replaySource: 'internal',
  simRunning: false,
  simError: null,
  replayT: 0,
  replayPlaying: false,
  replaySpeed: 1,
  compareB: { productId: '', payloadId: 'none' },
  envelopeAxis: 'altitude',
  envelopeOverlay: true,

  set: (p) => set(p),
  setEnv: (p) => set({ env: { ...get().env, ...p }, scenarioId: null }),
  setSim: (p) => set({ sim: { ...get().sim, ...p } }),
  applyScenario: (s) => {
    const product = get().specs?.products.find((p) => p.id === get().productId);
    const fold = product ? foldForMode(s.config.flightMode, product.geometry.hinge.maxFoldDeg) : get().foldDeg;
    set({
      scenarioId: s.id,
      env: { ...s.env },
      flightMode: s.config.flightMode,
      cruiseMotors: s.config.cruiseMotors,
      batterySoc: s.config.batterySoc,
      sim: { ...s.sim },
      foldDeg: fold,
      trace: null,
      replayT: 0,
    });
  },
  selectProduct: (id) => {
    const st = get();
    const product = st.specs?.products.find((p) => p.id === id);
    if (!product) return;
    const keep = product.payloadOptions.some((o) => o.id === st.payloadId);
    set({
      productId: id,
      payloadId: keep ? st.payloadId : product.payloadOptions[0]?.id ?? 'none',
      foldDeg: foldForMode(st.flightMode, product.geometry.hinge.maxFoldDeg),
      selectedPart: null,
      isolatedPart: null,
      hiddenParts: [],
      trace: null,
      replayT: 0,
    });
  },
  togglePart: (id) => {
    const h = new Set(get().hiddenParts);
    if (h.has(id)) h.delete(id);
    else h.add(id);
    set({ hiddenParts: [...h] });
  },
  requestCamera: (preset) => set({ cameraRequest: { preset, n: (get().cameraRequest?.n ?? 0) + 1 } }),
}));

export function foldForMode(mode: FlightMode, maxFoldDeg: number) {
  if (mode === 'hover') return maxFoldDeg;
  if (mode === 'cruise') return 0;
  return maxFoldDeg / 2;
}

/** Active trace for replay (internal simulation or imported CSV). */
export function activeTrace(s: Pick<AppState, 'trace' | 'externalTrace' | 'replaySource'>): SimTrace | null {
  if (s.replaySource === 'external') return s.externalTrace ?? s.trace;
  return s.trace ?? s.externalTrace;
}

/** Frame index for the replay time. */
export function frameIndex(trace: SimTrace, t: number): number {
  const arr = trace.t;
  if (!arr.length) return 0;
  let lo = 0, hi = arr.length - 1;
  if (t <= arr[0]) return 0;
  if (t >= arr[hi]) return hi;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] <= t) lo = mid;
    else hi = mid;
  }
  return lo;
}
