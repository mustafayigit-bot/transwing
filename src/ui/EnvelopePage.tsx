import { useEffect, useMemo, useState } from 'react';
import type { EnvelopeResult } from '../physics/envelope';
import { payloadById } from '../physics/massProps';
import { windAtHeight } from '../physics/wind';
import { runEnvelope } from '../lib/simClient';
import { useApp } from '../store/app';
import { AssumptionList, Btn, Select, Toggle } from './common';

const CLASS_COLORS = ['#10b981', '#f59e0b', '#ef4444'];
const CLASS_LABELS = ['Uçulabilir', 'Marjinal', 'Uçulamaz'];
const PRODUCT_COLORS = ['#38bdf8', '#0ea5e9', '#f472b6', '#db2777', '#a3e635', '#65a30d'];

export function EnvelopePage() {
  const s = useApp();
  const specs = s.specs!;
  const [dirMode, setDirMode] = useState<'scenario' | 'worst'>('scenario');
  const [results, setResults] = useState<{ value: EnvelopeResult; assumptions: string[] }[] | null>(null);
  const [busy, setBusy] = useState(false);
  const product = specs.products.find((p) => p.id === s.productId)!;
  const winds = useMemo(() => Array.from({ length: 31 }, (_, i) => i), []);
  const rowsFor = (pMax: number) =>
    s.envelopeAxis === 'altitude' ? Array.from({ length: 21 }, (_, i) => i * 200) : Array.from({ length: 21 }, (_, i) => (pMax * 1.5 * i) / 20);

  useEffect(() => {
    let cancelled = false;
    setBusy(true);
    const list = s.envelopeOverlay ? specs.products : [product];
    runEnvelope(
      list.map((p) => ({
        product: p,
        payload: payloadById(p, p.id === product.id ? s.payloadId : p.payloadOptions[0]?.id),
        axis: s.envelopeAxis,
        windsMs: winds,
        rows: rowsFor(p.mass.payloadMaxKg),
        altitudeM: s.env.altitudeM,
        isaDeltaK: s.env.isaDeltaK,
        windFromDeg: dirMode === 'worst' ? 'worst' : s.env.windFromDeg,
        controlReserveFraction: specs.mission.controlReserveFraction,
      })),
    ).then((r) => {
      if (!cancelled) {
        setResults(r);
        setBusy(false);
      }
    });
    return () => { cancelled = true; };
  }, [s.envelopeAxis, s.envelopeOverlay, s.productId, s.payloadId, s.env.altitudeM, s.env.isaDeltaK, s.env.windFromDeg, dirMode, specs]); // eslint-disable-line react-hooks/exhaustive-deps

  const main = results?.find((r) => r.value.productId === product.id);
  const rows = rowsFor(product.mass.payloadMaxKg);
  const W = 760, H = 440, padL = 64, padB = 44, padT = 16, padR = 16;
  const cw = (W - padL - padR) / winds.length;
  const ch = (H - padT - padB) / rows.length;
  const xOf = (w: number) => padL + (w / winds[winds.length - 1]) * (W - padL - padR - cw) + cw / 2;
  const rowLabel = (v: number) => (s.envelopeAxis === 'altitude' ? `${v.toFixed(0)} m` : `${v.toFixed(1)} kg`);
  const rowIndexY = (i: number) => H - padB - (i + 0.5) * ch;
  const effW = windAtHeight(s.env.windSpeedMs, specs.mission.hoverHeightM, s.env.logProfile);

  return (
    <div className="flex h-full flex-col overflow-auto p-4">
      <div className="mb-3 flex flex-wrap items-center gap-3">
        <h2 className="text-base font-semibold text-slate-100">Operasyonel zarf — {product.name}</h2>
        <div className="w-56">
          <Select value={s.envelopeAxis} onChange={(v) => s.set({ envelopeAxis: v })} options={[{ value: 'altitude', label: 'Rüzgâr × irtifa' }, { value: 'payload', label: 'Rüzgâr × payload kütlesi' }]} />
        </div>
        <div className="w-56">
          <Select value={dirMode} onChange={setDirMode} options={[{ value: 'scenario', label: `Senaryo yönü (${s.env.windFromDeg}°)` }, { value: 'worst', label: 'En kötü yön (0–180°)' }]} />
        </div>
        <Toggle label="6 ürünün sınırlarını üst üste göster" checked={s.envelopeOverlay} onChange={(v) => s.set({ envelopeOverlay: v })} />
        {busy && <span className="text-xs text-sky-300">Hesaplanıyor (Web Worker)…</span>}
      </div>
      <div className="flex flex-wrap gap-4">
        <svg viewBox={`0 0 ${W} ${H}`} className="w-full max-w-[900px] rounded-lg border border-slate-800 bg-slate-950">
          {main?.value.cells.map((line, i) =>
            line.map((k, j) => (
              <rect key={`${i}-${j}`} x={padL + j * cw} y={H - padB - (i + 1) * ch} width={cw + 0.5} height={ch + 0.5} fill={CLASS_COLORS[k]} opacity={s.envelopeOverlay ? 0.45 : 0.75}>
                <title>{`${winds[j]} m/s, ${rowLabel(rows[i])}: ${CLASS_LABELS[k]}`}</title>
              </rect>
            )),
          )}
          {/* axes */}
          {winds.filter((w) => w % 5 === 0).map((w) => (
            <g key={w}>
              <line x1={xOf(w)} x2={xOf(w)} y1={padT} y2={H - padB} stroke="#1e293b" />
              <text x={xOf(w)} y={H - padB + 14} fontSize={10} fill="#94a3b8" textAnchor="middle">{w}</text>
            </g>
          ))}
          <text x={padL + (W - padL - padR) / 2} y={H - 8} fontSize={11} fill="#94a3b8" textAnchor="middle">Hover yüksekliğinde ortalama rüzgâr [m/s]</text>
          {rows.map((r, i) => (i % 4 === 0 ? <text key={i} x={padL - 6} y={rowIndexY(i) + 3} fontSize={10} fill="#94a3b8" textAnchor="end">{rowLabel(r)}</text> : null))}
          <text x={14} y={padT + (H - padT - padB) / 2} fontSize={11} fill="#94a3b8" textAnchor="middle" transform={`rotate(-90 14 ${padT + (H - padT - padB) / 2})`}>
            {s.envelopeAxis === 'altitude' ? 'İrtifa (MSL) [m]' : 'Payload kütlesi [kg]'}
          </text>
          {/* product boundaries */}
          {s.envelopeOverlay && results?.map((r) => {
            const idx = specs.products.findIndex((p) => p.id === r.value.productId);
            const pts = r.value.marginalLimit.map((w, i) => `${w < 0 ? padL : xOf(w) + cw / 2},${rowIndexY(i)}`).join(' ');
            return <polyline key={r.value.productId} points={pts} fill="none" stroke={PRODUCT_COLORS[idx % PRODUCT_COLORS.length]} strokeWidth={r.value.productId === product.id ? 3 : 1.6} strokeDasharray={specs.products[idx].variant === 'military' ? '6 3' : undefined} />;
          })}
          {/* current condition marker */}
          {s.envelopeAxis === 'altitude' && effW <= winds[winds.length - 1] && (
            <circle cx={xOf(effW)} cy={rowIndexY(s.env.altitudeM / 200)} r={5} fill="none" stroke="#fff" strokeWidth={2}>
              <title>Mevcut senaryo</title>
            </circle>
          )}
        </svg>
        <div className="w-64 space-y-3 text-xs">
          <div className="space-y-1">
            {CLASS_LABELS.map((l, k) => (
              <div key={l} className="flex items-center gap-2"><span className="h-3 w-5 rounded" style={{ background: CLASS_COLORS[k] }} />{l}</div>
            ))}
          </div>
          {s.envelopeOverlay && (
            <div className="space-y-1">
              <div className="text-slate-400">Marjinal sınırı (ürün başına):</div>
              {specs.products.map((p, i) => (
                <button key={p.id} onClick={() => useApp.getState().selectProduct(p.id)} className={`flex w-full items-center gap-2 rounded px-1 text-left hover:bg-slate-800 ${p.id === product.id ? 'bg-slate-800' : ''}`}>
                  <svg width="24" height="6"><line x1="0" x2="24" y1="3" y2="3" stroke={PRODUCT_COLORS[i % PRODUCT_COLORS.length]} strokeWidth="2.5" strokeDasharray={p.variant === 'military' ? '6 3' : undefined} /></svg>
                  {p.id}
                </button>
              ))}
            </div>
          )}
          <Btn onClick={() => useApp.setState({ tab: 'config' })}>← Konfigüratöre dön</Btn>
          {main && <AssumptionList items={main.assumptions} />}
        </div>
      </div>
    </div>
  );
}
