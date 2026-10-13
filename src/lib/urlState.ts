/** Shareable URL: product, payload, scenario and view state are kept in the location hash. */
import { useApp, type AppState, type Tab } from '../store/app';
import type { TurbulenceLevel, FlightMode } from '../physics/types';

const TURB: TurbulenceLevel[] = ['none', 'light', 'moderate', 'severe'];
const MODES: FlightMode[] = ['hover', 'transition', 'cruise'];
const TABS: Tab[] = ['config', 'compare', 'envelope', 'matrix'];

export function encodeState(s: AppState): string {
  const q = new URLSearchParams();
  q.set('p', s.productId);
  q.set('pl', s.payloadId);
  if (s.scenarioId) q.set('sc', s.scenarioId);
  q.set('alt', String(s.env.altitudeM));
  q.set('dT', String(s.env.isaDeltaK));
  q.set('ws', String(s.env.windSpeedMs));
  q.set('wd', String(s.env.windFromDeg));
  q.set('tb', s.env.turbulence);
  q.set('ga', String(s.env.gustAmplitudeMs));
  q.set('lp', s.env.logProfile ? '1' : '0');
  q.set('fm', s.flightMode);
  q.set('cm', String(s.cruiseMotors));
  q.set('soc', String(s.batterySoc));
  q.set('fold', s.foldDeg.toFixed(1));
  q.set('ex', s.explode.toFixed(2));
  q.set('tab', s.tab);
  q.set('dur', String(s.sim.durationS));
  q.set('seed', String(s.sim.seed));
  q.set('gs', String(s.sim.gustStartS));
  q.set('gd', String(s.sim.gustDurationS));
  if (s.compareB.productId) {
    q.set('pb', s.compareB.productId);
    q.set('plb', s.compareB.payloadId);
  }
  return q.toString();
}

export function decodeState(hash: string, s: AppState): Partial<AppState> {
  const q = new URLSearchParams(hash.replace(/^#/, ''));
  const out: Partial<AppState> = {};
  const num = (k: string, d: number) => {
    const v = q.get(k);
    const n = v === null ? NaN : Number(v);
    return Number.isFinite(n) ? n : d;
  };
  const products = s.specs?.products ?? [];
  const p = q.get('p');
  if (p && products.some((x) => x.id === p)) out.productId = p;
  if (q.get('pl')) out.payloadId = q.get('pl')!;
  const sc = q.get('sc');
  if (sc && s.specs?.scenarios.some((x) => x.id === sc)) out.scenarioId = sc;
  const tb = q.get('tb') as TurbulenceLevel | null;
  out.env = {
    altitudeM: num('alt', s.env.altitudeM),
    isaDeltaK: num('dT', s.env.isaDeltaK),
    windSpeedMs: num('ws', s.env.windSpeedMs),
    windFromDeg: num('wd', s.env.windFromDeg),
    turbulence: tb && TURB.includes(tb) ? tb : s.env.turbulence,
    gustAmplitudeMs: num('ga', s.env.gustAmplitudeMs),
    logProfile: q.has('lp') ? q.get('lp') === '1' : s.env.logProfile,
  };
  const fm = q.get('fm') as FlightMode | null;
  if (fm && MODES.includes(fm)) out.flightMode = fm;
  const cm = num('cm', s.cruiseMotors);
  out.cruiseMotors = cm === 4 ? 4 : 2;
  out.batterySoc = Math.min(1, Math.max(0, num('soc', s.batterySoc)));
  if (q.has('fold')) out.foldDeg = num('fold', s.foldDeg);
  out.explode = Math.min(1, Math.max(0, num('ex', 0)));
  const tab = q.get('tab') as Tab | null;
  if (tab && TABS.includes(tab)) out.tab = tab;
  out.sim = {
    durationS: num('dur', s.sim.durationS),
    seed: Math.round(num('seed', s.sim.seed)),
    gustStartS: num('gs', s.sim.gustStartS),
    gustDurationS: num('gd', s.sim.gustDurationS),
  };
  const pb = q.get('pb');
  if (pb && products.some((x) => x.id === pb)) out.compareB = { productId: pb, payloadId: q.get('plb') ?? 'none' };
  return out;
}

let timer: ReturnType<typeof setTimeout> | null = null;

/** Keep the hash in sync with the store (debounced, replaceState so history is not spammed). */
export function startUrlSync() {
  return useApp.subscribe((s) => {
    if (!s.specs || !s.productId) return;
    if (timer) clearTimeout(timer);
    timer = setTimeout(() => {
      const h = '#' + encodeState(useApp.getState());
      if (h !== window.location.hash) window.history.replaceState(null, '', h);
    }, 250);
  });
}

export function shareUrl(): string {
  return `${window.location.origin}${window.location.pathname}#${encodeState(useApp.getState())}`;
}
