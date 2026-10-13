/** Mission energy budget: take-off hover + transition + cruise + transition + landing hover + reserve. */
import type { MissionSpec, PayloadOption, Product } from '../data/schema';
import { cruisePerformance } from './cruise';
import { hoverPerformance } from './hover';
import type { Result } from './types';

/** Average transition power relative to hover power. */
export const TRANSITION_POWER_FACTOR = 0.85;

export interface MissionInput {
  product: Product;
  payload: PayloadOption | null;
  mission: MissionSpec;
  altitudeM: number;
  isaDeltaK: number;
  motors: 2 | 4;
  batterySoc: number;
  /** Mean wind speed and direction (from, relative to nose) — head/tail-wind component affects range. */
  windSpeedMs: number;
  windFromDeg: number;
}

export interface MissionResult {
  energyAvailableWh: number;
  reserveWh: number;
  hoverEnergyWh: number;
  transitionEnergyWh: number;
  cruiseEnergyWh: number;
  /** Total flight time with cruise at minimum power speed (loiter), s */
  enduranceS: number;
  /** One-way range at best-range speed including VTOL phases, m */
  rangeM: number;
  /** Pure hover endurance to reserve, s */
  hoverEnduranceS: number;
  headwindMs: number;
  groundSpeedMs: number;
  feasible: boolean;
  phases: { name: string; durationS: number; powerW: number; energyWh: number }[];
}

export function missionPerformance(input: MissionInput): Result<MissionResult> {
  const { product, mission } = input;
  const hover = hoverPerformance({ product, payload: input.payload, altitudeM: input.altitudeM, isaDeltaK: input.isaDeltaK });
  const cruise = cruisePerformance({ product, payload: input.payload, altitudeM: input.altitudeM, isaDeltaK: input.isaDeltaK, motors: input.motors });
  const usableWh = product.energy.batteryWh * product.energy.usableFraction;
  const energyAvailableWh = usableWh * input.batterySoc;
  const reserveWh = usableWh * mission.reserveFraction;
  const pHover = hover.value.totalElectricPowerW;
  const hoverS = mission.takeoffHoverS + mission.landingHoverS;
  const hoverEnergyWh = (pHover * hoverS) / 3600;
  const transS = 2 * mission.transitionS;
  const transitionEnergyWh = (TRANSITION_POWER_FACTOR * pHover * transS) / 3600;
  const cruiseEnergyWh = Math.max(0, energyAvailableWh - reserveWh - hoverEnergyWh - transitionEnergyWh);
  const c = cruise.value;
  const headwind = input.windSpeedMs * Math.cos((input.windFromDeg * Math.PI) / 180);
  const gs = Math.max(0, c.bestRangeSpeedMs - headwind);
  const cruiseTimeBr = (cruiseEnergyWh * 3600) / c.bestRangeElectricW;
  const rangeM = cruiseTimeBr * gs + 0; // VTOL phases assumed stationary
  const loiterS = (cruiseEnergyWh * 3600) / c.minPowerElectricW;
  const enduranceS = hoverS + transS + loiterS;
  const hoverEnduranceS = (Math.max(0, energyAvailableWh - reserveWh) * 3600) / pHover;
  return {
    value: {
      energyAvailableWh,
      reserveWh,
      hoverEnergyWh,
      transitionEnergyWh,
      cruiseEnergyWh,
      enduranceS,
      rangeM,
      hoverEnduranceS,
      headwindMs: headwind,
      groundSpeedMs: gs,
      feasible: energyAvailableWh - reserveWh > hoverEnergyWh + transitionEnergyWh && hover.value.powerMargin > 0,
      phases: [
        { name: 'Kalkış hover', durationS: mission.takeoffHoverS, powerW: pHover, energyWh: (pHover * mission.takeoffHoverS) / 3600 },
        { name: 'Geçiş ×2', durationS: transS, powerW: TRANSITION_POWER_FACTOR * pHover, energyWh: transitionEnergyWh },
        { name: 'Cruise (en iyi menzil)', durationS: cruiseTimeBr, powerW: c.bestRangeElectricW, energyWh: cruiseEnergyWh },
        { name: 'İniş hover', durationS: mission.landingHoverS, powerW: pHover, energyWh: (pHover * mission.landingHoverS) / 3600 },
        { name: 'Rezerv', durationS: 0, powerW: 0, energyWh: reserveWh },
      ],
    },
    assumptions: [
      `Kullanılabilir enerji = batteryWh × usableFraction × doluluk; rezerv = kullanılabilir enerjinin %${Math.round(mission.reserveFraction * 100)}'i.`,
      `Geçiş gücü hover gücünün %${Math.round(TRANSITION_POWER_FACTOR * 100)}'i, iki geçiş (kalkış ve iniş).`,
      'Menzil: en iyi menzil hızında, rüzgârın karşı bileşeni yer hızından düşülerek (tek yön).',
      'Dayanım: cruise kısmı minimum güç hızında bekleme (loiter) olarak hesaplanır.',
      'Batarya gerilim düşümü, sıcaklık ve yaşlanma etkileri ihmal edildi; sabit kullanılabilir kapasite.',
      'Aviyonik / payload elektrik tüketimi ihmal edildi.',
      ...hover.assumptions,
      ...cruise.assumptions,
    ].filter((v, i, a) => a.indexOf(v) === i),
  };
}
