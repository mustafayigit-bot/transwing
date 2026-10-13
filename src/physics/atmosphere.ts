/** ISA standard atmosphere (troposphere + lower stratosphere) with temperature offset. */
import type { Result } from './types';

export const ISA = {
  T0K: 288.15,
  P0Pa: 101325,
  rho0: 1.225,
  lapseKPerM: 0.0065,
  R: 287.05287,
  g0: 9.80665,
  tropopauseM: 11000,
};

export interface AtmosphereState {
  temperatureK: number;
  pressurePa: number;
  densityKgM3: number;
  densityRatio: number;
  speedOfSoundMs: number;
}

/**
 * Pressure follows the standard (ISA) profile at the given geopotential altitude; the ISA deviation
 * only changes the temperature and therefore the density (common performance convention).
 */
export function atmosphere(altitudeM: number, isaDeltaK = 0): Result<AtmosphereState> {
  const { T0K, P0Pa, lapseKPerM, R, g0, tropopauseM } = ISA;
  const h = Math.max(-500, altitudeM);
  let tStd: number;
  let p: number;
  if (h <= tropopauseM) {
    tStd = T0K - lapseKPerM * h;
    p = P0Pa * Math.pow(tStd / T0K, g0 / (lapseKPerM * R));
  } else {
    const t11 = T0K - lapseKPerM * tropopauseM;
    const p11 = P0Pa * Math.pow(t11 / T0K, g0 / (lapseKPerM * R));
    tStd = t11;
    p = p11 * Math.exp((-g0 * (h - tropopauseM)) / (R * t11));
  }
  const t = tStd + isaDeltaK;
  const rho = p / (R * t);
  return {
    value: {
      temperatureK: t,
      pressurePa: p,
      densityKgM3: rho,
      densityRatio: rho / ISA.rho0,
      speedOfSoundMs: Math.sqrt(1.4 * R * t),
    },
    assumptions: [
      'ISA modeli (ICAO), kuru hava, R = 287.05 J/(kg·K).',
      'ISA sapması yalnızca sıcaklığı değiştirir; basınç standart profildedir.',
      'Nem etkisi ihmal edildi.',
    ],
  };
}
