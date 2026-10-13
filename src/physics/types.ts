/** Every physics function returns its value together with the modelling assumptions used. */
export interface Result<T> {
  value: T;
  assumptions: string[];
}

export const G0 = 9.80665;

export type FlightMode = 'hover' | 'transition' | 'cruise';
export type TurbulenceLevel = 'none' | 'light' | 'moderate' | 'severe';

export interface Environment {
  altitudeM: number;
  isaDeltaK: number;
  windSpeedMs: number;
  windFromDeg: number;
  turbulence: TurbulenceLevel;
  gustAmplitudeMs: number;
  logProfile: boolean;
}

export interface Configuration {
  productId: string;
  payloadId: string;
  flightMode: FlightMode;
  cruiseMotors: 2 | 4;
  batterySoc: number;
  foldDeg: number;
}
