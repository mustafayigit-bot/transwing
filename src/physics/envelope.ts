/** Operational envelope sweeps: wind × altitude or wind × payload → flyable / marginal / not flyable. */
import type { PayloadOption, Product } from '../data/schema';
import { hoverWindCheck, MAX_HOVER_TILT_DEG } from './hover';
import type { Result } from './types';

export type EnvelopeClass = 0 | 1 | 2; // 0 flyable, 1 marginal, 2 not flyable

export interface EnvelopeInput {
  product: Product;
  /** Payload option whose drag area is used; mass may be overridden per row in payload mode. */
  payload: PayloadOption | null;
  axis: 'altitude' | 'payload';
  windsMs: number[];
  /** altitude values (m) or payload masses (kg) */
  rows: number[];
  altitudeM: number;
  isaDeltaK: number;
  windFromDeg: number | 'worst';
  controlReserveFraction: number;
}

export interface EnvelopeResult {
  productId: string;
  cells: EnvelopeClass[][]; // [row][wind]
  /** highest wind (m/s) still flyable / at most marginal per row */
  flyableLimit: number[];
  marginalLimit: number[];
}

export const MARGINAL_TILT_FRACTION = 0.7;
export const MARGINAL_UTILISATION = 0.85;

export function classifyHover(input: Parameters<typeof hoverWindCheck>[0], windMs: number, dirs: number[]): EnvelopeClass {
  let worst: EnvelopeClass = 0;
  for (const d of dirs) {
    const c = hoverWindCheck(input, windMs, d);
    let k: EnvelopeClass = 0;
    if (c.limitedBy !== 'ok') k = 2;
    else if (c.tiltDeg > MARGINAL_TILT_FRACTION * MAX_HOVER_TILT_DEG || Math.max(...c.rotorUtilization) > MARGINAL_UTILISATION) k = 1;
    if (k > worst) worst = k;
    if (worst === 2) break;
  }
  return worst;
}

export function sweepEnvelope(inp: EnvelopeInput): Result<EnvelopeResult> {
  const dirs = inp.windFromDeg === 'worst' ? [0, 45, 90, 135, 180] : [inp.windFromDeg];
  const cells: EnvelopeClass[][] = [];
  for (const row of inp.rows) {
    const payload: PayloadOption | null =
      inp.axis === 'payload'
        ? { id: '_sweep', name: 'sweep', massKg: row, dragAreaM2: inp.payload?.dragAreaM2 ?? 0, variant: 'both', shape: 'box', estimate: true }
        : inp.payload;
    const base = {
      product: inp.product,
      payload,
      altitudeM: inp.axis === 'altitude' ? row : inp.altitudeM,
      isaDeltaK: inp.isaDeltaK,
      controlReserveFraction: inp.controlReserveFraction,
    };
    const line: EnvelopeClass[] = [];
    let stop = false;
    for (const w of inp.windsMs) {
      // classes are monotonic in wind speed, so once infeasible the rest of the row is too
      const k = stop ? 2 : classifyHover(base, w, dirs);
      if (k === 2) stop = true;
      line.push(k);
    }
    cells.push(line);
  }
  const limit = (maxClass: EnvelopeClass) =>
    cells.map((line) => {
      let v = -1;
      line.forEach((k, j) => { if (k <= maxClass) v = inp.windsMs[j]; });
      return v;
    });
  return {
    value: { productId: inp.product.id, cells, flyableLimit: limit(0), marginalLimit: limit(1) },
    assumptions: [
      'Zarf, hover pozisyon tutma statik analizine dayanır (eğim, itki ve moment otoritesi kontrolleri).',
      `Marjinal: eğim > ${MARGINAL_TILT_FRACTION * 100}% × ${MAX_HOVER_TILT_DEG}° veya en yüklü rotor kullanılabilir itkinin > %${MARGINAL_UTILISATION * 100}'i.`,
      inp.windFromDeg === 'worst' ? 'Rüzgâr yönü: 0–180° arasında 45° adımla en kötü yön.' : `Rüzgâr yönü: ${inp.windFromDeg}° (buruna göre, geldiği yön).`,
      'Rüzgâr değerleri hover yüksekliğindeki ortalama rüzgârdır (gerçek hava hızı).',
      'Eğim sınırlı bölgede limit irtifayla artabilir: D ∝ ρV² olduğundan seyrek havada aynı eğim daha yüksek gerçek rüzgâr hızına karşılık gelir; itki sınırı ise ρ ile azalır.',
    ],
  };
}
