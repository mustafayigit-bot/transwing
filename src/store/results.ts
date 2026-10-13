import { useMemo } from 'react';
import type { MissionSpec, PayloadOption, Product } from '../data/schema';
import { cruisePerformance } from '../physics/cruise';
import { hoverPerformance, hoverWindLimit, MAX_HOVER_TILT_DEG } from '../physics/hover';
import { massProperties, payloadById } from '../physics/massProps';
import { missionPerformance } from '../physics/mission';
import type { Environment } from '../physics/types';
import { windAtHeight } from '../physics/wind';
import { useApp } from './app';

export type Status = 'green' | 'yellow' | 'red' | 'neutral';

export interface ResultCard {
  id: string;
  label: string;
  value: string;
  sub?: string;
  status: Status;
  assumptions: string[];
  raw: number;
}

export interface EvalInput {
  product: Product;
  payload: PayloadOption | null;
  env: Environment;
  mission: MissionSpec;
  cruiseMotors: 2 | 4;
  batterySoc: number;
}

/** Full static evaluation of a configuration (pure; also used by compare, matrix and export). */
export function evaluate(i: EvalInput) {
  const { product, payload, env, mission } = i;
  const hoverIn = { product, payload, altitudeM: env.altitudeM, isaDeltaK: env.isaDeltaK, controlReserveFraction: mission.controlReserveFraction };
  const hover = hoverPerformance(hoverIn);
  const cruise = cruisePerformance({ ...hoverIn, motors: i.cruiseMotors });
  const effWind = windAtHeight(env.windSpeedMs, mission.hoverHeightM, env.logProfile);
  const windLimit = hoverWindLimit(hoverIn, effWind, env.windFromDeg);
  const mission_ = missionPerformance({
    product, payload, mission, altitudeM: env.altitudeM, isaDeltaK: env.isaDeltaK, motors: i.cruiseMotors,
    batterySoc: i.batterySoc, windSpeedMs: windAtHeight(env.windSpeedMs, 100, env.logProfile), windFromDeg: env.windFromDeg,
  });
  const massCruise = massProperties(product, payload, 0);
  const massHover = massProperties(product, payload, product.geometry.hinge.maxFoldDeg);
  return { hover, cruise, windLimit, mission: mission_, effWind, massCruise, massHover };
}
export type Evaluation = ReturnType<typeof evaluate>;

const pct = (x: number) => `${(x * 100).toFixed(0)} %`;

export function buildCards(ev: Evaluation): ResultCard[] {
  const h = ev.hover.value, c = ev.cruise.value, m = ev.mission.value, w = ev.windLimit.value;
  const tilt = w.atScenario.tiltDeg;
  const windRatio = w.maxWindMs > 0 ? ev.effWind / w.maxWindMs : Infinity;
  const limitText: Record<string, string> = { ok: '—', tilt: 'eğim sınırı', thrust: 'itki', moment: 'moment otoritesi' };
  return [
    {
      id: 'powerMargin', label: 'Hover güç marjı', value: pct(h.powerMargin), raw: h.powerMargin,
      sub: `T/W ${h.thrustToWeight.toFixed(2)} · ${(h.totalElectricPowerW / 1000).toFixed(1)} kW`,
      status: h.powerMargin >= 0.3 ? 'green' : h.powerMargin >= 0.1 ? 'yellow' : 'red',
      assumptions: ['Eşikler: yeşil ≥ %30, sarı ≥ %10, kırmızı < %10.', ...ev.hover.assumptions],
    },
    {
      id: 'maxWind', label: 'Maks. hover rüzgârı', value: `${w.maxWindMs.toFixed(1)} m/s`, raw: w.maxWindMs,
      sub: `Mevcut ${ev.effWind.toFixed(1)} m/s · sınır: ${limitText[w.limitedBy]} · en kötü yön ${w.worstDirectionDeg}° (${w.worstDirectionMaxWindMs.toFixed(1)})`,
      status: w.atScenario.limitedBy !== 'ok' || windRatio > 1 ? 'red' : windRatio > 0.75 ? 'yellow' : 'green',
      assumptions: [
        'Mevcut rüzgâr, hover yüksekliğindeki (logaritmik profil) ortalama rüzgârdır.',
        'Eşikler: yeşil ≤ limitin %75’i, sarı ≤ limit, kırmızı > limit.',
        ...ev.windLimit.assumptions,
      ],
    },
    {
      id: 'endurance', label: 'Dayanım', value: `${(m.enduranceS / 60).toFixed(0)} dk`, raw: m.enduranceS,
      sub: `Saf hover: ${(m.hoverEnduranceS / 60).toFixed(1)} dk`,
      status: !m.feasible ? 'red' : m.enduranceS >= 40 * 60 ? 'green' : m.enduranceS >= 20 * 60 ? 'yellow' : 'red',
      assumptions: ['Eşikler: yeşil ≥ 40 dk, sarı ≥ 20 dk.', ...ev.mission.assumptions],
    },
    {
      id: 'range', label: 'Menzil', value: `${(m.rangeM / 1000).toFixed(0)} km`, raw: m.rangeM,
      sub: `${(m.rangeM / 1852).toFixed(0)} nm · Vbr ${c.bestRangeSpeedMs.toFixed(1)} m/s · yer hızı ${m.groundSpeedMs.toFixed(1)}`,
      status: !m.feasible ? 'red' : m.rangeM >= 60000 ? 'green' : m.rangeM >= 25000 ? 'yellow' : 'red',
      assumptions: ['Eşikler: yeşil ≥ 60 km, sarı ≥ 25 km.', ...ev.mission.assumptions],
    },
    {
      id: 'stall', label: 'Stall hızı', value: `${c.stallSpeedMs.toFixed(1)} m/s`, raw: c.stallSpeedMs,
      sub: `Vmp ${c.minPowerSpeedMs.toFixed(1)} · Vmax ${c.maxSpeedMs.toFixed(1)} m/s · (L/D)max ${c.bestLD.toFixed(1)}`,
      status: c.stallSpeedMs <= 25 ? 'green' : c.stallSpeedMs <= 32 ? 'yellow' : 'red',
      assumptions: ['Eşikler (geçiş hızı ihtiyacı): yeşil ≤ 25 m/s, sarı ≤ 32 m/s.', ...ev.cruise.assumptions],
    },
    {
      id: 'tilt', label: 'Gereken eğim açısı', value: `${tilt.toFixed(1)}°`, raw: tilt,
      sub: `Yan sürüklenme ${w.atScenario.dragN.toFixed(0)} N · alan ${w.atScenario.sideAreaM2.toFixed(2)} m²`,
      status: tilt >= MAX_HOVER_TILT_DEG ? 'red' : tilt >= 0.7 * MAX_HOVER_TILT_DEG ? 'yellow' : 'green',
      assumptions: [`Eşikler: kırmızı ≥ ${MAX_HOVER_TILT_DEG}°, sarı ≥ %70.`, ...ev.windLimit.assumptions],
    },
  ];
}

export function useCurrentProduct(): Product | null {
  const specs = useApp((s) => s.specs);
  const id = useApp((s) => s.productId);
  return useMemo(() => specs?.products.find((p) => p.id === id) ?? null, [specs, id]);
}

export function usePayload(product: Product | null, payloadId: string): PayloadOption | null {
  return useMemo(() => (product ? payloadById(product, payloadId) : null), [product, payloadId]);
}

export function useEvaluation() {
  const product = useCurrentProduct();
  const specs = useApp((s) => s.specs);
  const payloadId = useApp((s) => s.payloadId);
  const env = useApp((s) => s.env);
  const cruiseMotors = useApp((s) => s.cruiseMotors);
  const batterySoc = useApp((s) => s.batterySoc);
  const payload = usePayload(product, payloadId);
  return useMemo(() => {
    if (!product || !specs) return null;
    const ev = evaluate({ product, payload, env, mission: specs.mission, cruiseMotors, batterySoc });
    return { ev, cards: buildCards(ev), product, payload };
  }, [product, payload, env, specs, cruiseMotors, batterySoc]);
}
